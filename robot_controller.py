import json
import sys
import math
import threading
import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List, Optional
from queue import Queue, Empty

from actions import actions
from routes import routes
from available_trajectories import available_trajectories
from config import POINTS_PATH, TRAJ_PATH, NUM_DIGITAL_IO, GRIPPER_DO_INDEX
from commands import Command, CmdType, RobotTrajectories, RobotActions, RobotRoutes

# sys.path.append("/home/user/robot-api")
sys.path.append("robot-api")
from API.source.core.exceptions.data_validation_error.generic_error import (
    AddWaypointError, FunctionTimeOutError)
from API.source.models.classes.enum_classes.state_classes import (
    InComingControllerState as Ics,
    InComingSafetyStatus as Iss)


@dataclass
class RobotState:
    controller_state: str = "unknown"
    safety_status: str = "unknown"
    tcp_position: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_position: List[float] = field(default_factory=lambda: [0.0] * 6)
    mode: str = "hold"  # 'hold' | 'move' | ...
    powered: bool = False
    free_drive: bool = False
    last_error: Optional[str] = None
    last_command: Optional[str] = None
    cmd_state: int = 0


class RobotController:

    def __init__(self, robot_ip: str, cmd_queue: Queue, logger, heartbeat_cb=None):
        self.log = logger
        # раскомментить для отладки с манипулятором по месту
        # try:
        #     self.Robot = RobotApi(robot_ip, show_std_traceback=True)
        # except VersionError as e:
        #     self.log.critical(f"Robot API version mismatch: {e}")
        #     raise
        self.cmd_queue = cmd_queue
        self.stop_event = threading.Event()

        self._state_lock = threading.Lock()
        self._state = RobotState()

        self.Waypoints: Dict[str, Dict] = self.load_waypoints()
        self.Trajectories = self.load_trajectories()
        self.Actions = actions
        self.Routes = routes

        self._heartbeat_cb = heartbeat_cb
        self._nearest_info = None
        self._nearest_boot_done = False
        self._joystick_running = False
        self._joystick_thread = None

    @staticmethod
    def _normalize_ratio(v) -> float:
        try:
            x = float(v)

        except Exception:
            return 0.0
        if math.isnan(x) or math.isinf(x):
            return 0.0
        if x < 0.0:
            return 0.0
        if x > 1.0:
            return 1.0
        return x

    # ---------- Вспомогательные методы состояния ----------
    def _set_state(self, **kwargs):
        with self._state_lock:
            valid = {f.name for f in dataclasses.fields(self._state)}
            for k, v in kwargs.items():
                if k not in valid:
                    raise KeyError(f"Unknown state field: {k}")
                setattr(self._state, k, v)

    def get_state_snapshot(self) -> RobotState:
        with self._state_lock:
            return RobotState(**self._state.__dict__)

    def get_waypoints_snapshot(self) -> dict:
        return dict(self.Waypoints)

    def get_trajectories_snapshot(self) -> dict:
        return dict(self.Trajectories)

    def get_actions_snapshot(self) -> dict:
        return dict(self.Actions)

    def get_current_tcp_position(self) -> List[float]:
        with self._state_lock:
            return list(self._state.tcp_position)

    def get_current_joint_position(self) -> List[float]:
        with self._state_lock:
            return list(self._state.joint_position)

    def get_controller_state(self) -> str:
        with self._state_lock:
            return self._state.controller_state

    def get_nearest_info(self):
        return self._nearest_info or None

    def find_nearest_waypoint(self) -> dict:
        current_tcp = self.get_current_tcp_position()

        self.refresh_waypoints()
        self.Trajectories = self.load_trajectories()
        best_name = None
        best_dist = float("inf")

        for name, wp in self.Waypoints.items():
            tcp = wp.get("tcp")

            dist = math.sqrt((current_tcp[0] - float(tcp["x"])) ** 2 +
                             (current_tcp[1] - float(tcp["y"])) ** 2 +
                             (current_tcp[2] - float(tcp["z"])) ** 2)
            if dist < best_dist:
                best_dist, best_name = dist, name
        if best_name is None:
            info = {"waypoint": "", "distance": 0.0, "trajectories": []}
            self._nearest_info = info
            return info

        traj_list = []
        for traj_name, traj in self.Trajectories.items():
            for entry in traj.get("positions", []):
                if entry.get("name") == best_name:
                    traj_list.append(traj_name)
                    break
        info = {"waypoint": best_name, "distance": best_dist,
                "trajectories": traj_list}
        self._nearest_info = info
        self.log.info(f"Nearest waypoint TCP: {best_name}; "
                      f"distance={best_dist:.3f}; in trajectories={traj_list}")
        return info

    def _set_cmd_state(self, code: int) -> None:
        with self._state_lock:
            self._state.cmd_state = code

    def _joystick_worker(self, coord_sys=None) -> None:
        self._joystick_running = True
        self.log.info("simple joystick started")
        try:
            if coord_sys is None:
                self.Robot.motion.simple_joystick()
            else:
                self.Robot.motion.simple_joystick(coordinate_system=coord_sys)
        finally:
            self._joystick_running = False
            self.log.info("simple joystick finished")

    def wait_motion_complete(self, await_sec: int = -1) -> bool:
        try:
            return (self.Robot.motion.
                    wait_waypoint_completion(0, await_sec=await_sec))
        except Exception as e:
            self.log.error(f"wait motion complete failed: {e}")
            return False

    # ---------- Работа с данными ----------
    def load_waypoints(self) -> Dict[str, Dict]:
        try:
            with open(POINTS_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return {wp['name']: wp for wp in data['waypoints']}
        except Exception as e:
            self.log.warning(f"Error loading positions from {POINTS_PATH}: {e}")
            return {}

    def load_trajectories(self) -> Dict[str, Dict]:
        try:
            with open(TRAJ_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return {traj['name']: traj for traj in data['trajectories']}
        except Exception as e:
            self.log.warning(f"Error loading trajectories from {TRAJ_PATH}: {e}")
            return {}

    def refresh_waypoints(self):
        self.Waypoints = self.load_waypoints()

    # ---------- Операции движения/IO ----------
    def add_waypoint_line(self,
                          tcp_pose: list,
                          speed: float,
                          accel: float,
                          blend: float):
        n_speed = self._normalize_ratio(speed)
        n_accel = self._normalize_ratio(accel)

        if (n_speed != float(speed)) or (n_accel != float(accel)):
            self.log.debug(f"Line motion params normalized: "
                           f"speed {speed}→{n_speed}, accel {accel}→{n_accel}")

        self.Robot.motion.linear.add_new_waypoint(
            tcp_pose=tcp_pose,
            speed=n_speed,
            accel=n_accel,
            blend=blend,
            orientation_units='deg')

    def add_waypoint_joint(self,
                           angle_pose: List[float],
                           speed: float,
                           accel: float,
                           blend: float):
        self.Robot.motion.joint.add_new_waypoint(
            angle_pose=angle_pose,
            speed=speed,
            accel=accel,
            blend=blend,
            units='deg'
        )

    def get_tcp_pose(self, waypoint_name: str) -> List[float]:
        wp = self.Waypoints.get(waypoint_name)

        if not wp:
            raise ValueError(f"Waypoint {waypoint_name} not found")
        pos = wp.get('tcp')
        return [pos['x'], pos['y'], pos['z'], pos['Rx'], pos['Ry'], pos['Rz']]

    def get_joint_pose(self, waypoint_name: str) -> List[float]:
        wp = self.Waypoints.get(waypoint_name)

        if not wp:
            raise ValueError(f"Waypoint {waypoint_name} not found")
        joints = wp.get('joints')
        return [joints['J1'], joints['J2'], joints['J3'], joints['J4'], joints['J5'], joints['J6']]

    def get_move_params(self, waypoint_name: str) -> Dict[str, float]:
        wp = self.Waypoints.get(waypoint_name)
        if not wp:
            raise ValueError(f"Waypoint {waypoint_name} not found")
        return {
            'speed': float(wp['speed']),
            'accel': float(wp['accel']),
            'blend': float(wp['blend'])
        }

    def get_waypoint_data(self, waypoint_name: str) -> Dict:
        return {
            'tcp_pose': self.get_tcp_pose(waypoint_name),
            'joint_pose': self.get_joint_pose(waypoint_name),
            'move_params': self.get_move_params(waypoint_name)
        }

    def control_digital_outputs(self, index: int, value: bool) -> bool:
        if not 0 <= index < NUM_DIGITAL_IO:
            self.log.error(f"Invalid output index: {index}")
            return False
        try:
            return self.Robot.io.digital.set_output(index=index, value=value)
        except Exception as e:
            self.log.error(f"Error setting output {index}: {e}")
            return False

    # ---------- Управление режимами ----------
    def manipulator_power_control(self, qPowerOn: int) -> None:
        try:
            if qPowerOn == 2:
                self.Robot.controller_state.set('off', await_sec=10)
                self._set_state(powered=False, last_command="PowerOff")
                self.log.info("Manipulator deactivated")
            elif qPowerOn == 1:
                if self.Robot.controller_state.get() != 'run':
                    self.Robot.controller_state.set('off', await_sec=1)
                    self.Robot.controller_state.set('run', await_sec=10)
                self._set_state(powered=True, last_command="PowerOn")
                self.log.info("Manipulator powered")
            else:
                self.log.warning(f"Unknown power command: {qPowerOn}")
        except FunctionTimeOutError as e:
            self._set_state(
                last_error=f"Timeout switching controller state: {e}")
            self.log.error(f"Timeout switching controller state: {e}")
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.error(f"Error in ManipulatorPowerControl: {e}")

    def manipulator_stop_drive(self) -> None:
        try:
            self.Robot.motion.mode.set('hold')
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.error(f"Stop Mode Switching Error: {e}")

    def manipulator_free_drive(self, qFreeDrive: int) -> None:
        try:
            if qFreeDrive == 1:
                self.log.info("Activating Zero Gravity Mode")
                self.Robot.motion.free_drive()
                self._set_state(free_drive=True, mode='hold')
            elif qFreeDrive == 2:
                self.log.info("Deactivating Zero Gravity Mode")
                self.Robot.motion.mode.set('hold')
                self._set_state(free_drive=False, mode='hold')
            else:
                self.log.warning(f"Unknown free drive command: {qFreeDrive}")
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.error(f"Zero Gravity Mode Switching Error: {e}")

    # Выполнение траектории по enum-команде
    def execute_trajectory(self, command: RobotTrajectories) -> None:
        try:
            if self.Robot.controller_state.get() != 'run':
                self.Robot.controller_state.set('off', await_sec=1)
                self.Robot.controller_state.set('run', await_sec=10)
            traj_name = command.name
            if traj_name not in self.Trajectories:
                raise ValueError(f"No trajectory mapped for command {command}")

            nearest_point = self.find_nearest_waypoint()
            nearest_wp = nearest_point.get("waypoint")
            print(nearest_wp)
            if traj_name not in available_trajectories[nearest_wp]:
                self._set_cmd_state(command.value + 400)
                self._set_state(last_error="Manipulator can't be moved by "
                                           "the selected trajectory from current point!")
                self.log.error("Manipulator can't be moved by "
                               "the selected trajectory from current point!")
            else:
                traj = self.Trajectories[traj_name]
                for position in traj['positions']:
                    self.cmd_queue.put(Command(CmdType.MOVE_TO_POINT,
                                               {"name": position.get("name"),
                                                "motion": "joint"},
                                               source="GUI"))

                self.Robot.motion.mode.set('move')
                self._set_state(mode='move', last_command=traj_name)
                self.log.info(f"Executing trajectory: {traj_name}")

                if command:
                    self._set_cmd_state(command.value + 100)

                finish_motion = self.wait_motion_complete(await_sec=-1)
                if command and finish_motion:
                    self._set_cmd_state(command.value + 200)

        except AddWaypointError as e:
            if command:
                self._set_cmd_state(command.value + 300)
            self._set_state(last_error=f"Add waypoint error: {e}")
            self.log.error(
                f"Error adding a movement point: {e}. Details: {e.args}")
        except FunctionTimeOutError as e:
            self._set_state(last_error=f"Timeout while executing command: {e}")
            self.log.error(f"Timeout while executing command: {e}")
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.error(f"ExecuteEnumCommand failed: {e}")

    def execute_action(self, action_name) -> None:
        for command in self.Actions.get(action_name).commands:
            if command.cmd_type == "EXECUTE_TRAJECTORY":
                self.cmd_queue.put(Command(
                    CmdType.EXECUTE_TRAJECTORY,
                    {'traj': int(getattr(RobotTrajectories, command.name))},
                    source="GUI"))
            if command.cmd_type == "IO_SET":
                self.cmd_queue.put(Command(
                    CmdType.IO_SET,
                    {'index': GRIPPER_DO_INDEX,
                     'value': bool(command.name)},
                    source="GUI"))

    def execute_route(self, route_name) -> None:
        for traj in self.Routes.get(route_name).trajectories:
            self.cmd_queue.put(Command(
                CmdType.EXECUTE_TRAJECTORY,
                {'traj': int(getattr(RobotTrajectories, traj.name))},
                source="GUI"))

    # Точка-в-точку по имени waypoint (для GUI)
    def move_to_point(self, point_name: str, motion: str = 'line') -> None:
        if self.Robot.controller_state.get() != 'run':
            self.Robot.controller_state.set('off', await_sec=1)
            self.Robot.controller_state.set('run', await_sec=10)

        if point_name not in self.Waypoints:
            raise ValueError(f"Waypoint {point_name} not found")
        data = self.get_waypoint_data(point_name)
        mp = data['move_params']

        if motion == 'joint':
            if not data.get('joint_pose'):
                raise ValueError(f"Waypoint {point_name} "
                                 f"has no joint angles for joint motion")
            self.add_waypoint_joint(data['joint_pose'],
                                    mp['speed'],
                                    mp['accel'],
                                    mp['blend'])
        else:
            self.add_waypoint_line(data['tcp_pose'],
                                   mp['speed'] / 100,
                                   mp['accel'] / 100,
                                   mp['blend'] / 100)

        self.Robot.motion.mode.set('move')
        self._set_state(mode='move',
                        last_command=f"MoveToPoint:{point_name}")
        self.log.info(f"Moving to point '{point_name}'")

    # simple_joystick
    def start_simple_joystick(self, coord_sys=None) -> None:
        if (self._joystick_running and self._joystick_thread and
                self._joystick_thread.is_alive()):
            self.log.info("simple joystick already running")
            return

        if self.Robot.controller_state.get() != 'run':
            self.Robot.controller_state.set('off', await_sec=1)
            self.Robot.controller_state.set('run', await_sec=10)

        self._joystick_thread = threading.Thread(
            target=self._joystick_worker,
            args=(coord_sys,),
            name="SimpleJoystickThread",
            daemon=True
        )
        self._joystick_thread.start()

    # ---------- Мониторинг/диагностика ----------
    def check_controller_state(self) -> None:
        try:
            safety = self.Robot.safety_status.get()
            ctrl = self.Robot.controller_state.get()
            self._set_state(safety_status=safety, controller_state=ctrl)
            if (safety == Iss.fault.name or ctrl == Ics.failure.name):
                self.log.error("Manipulator Error (fault/failure)")
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.error(f"CheckControllerState error: {e}")

    def _update_telemetry(self) -> None:
        try:
            tcp = self.Robot.motion.linear.get_actual_position()
            joints = self.Robot.motion.joint.get_actual_position()  # НОВОЕ
            mode = self.Robot.motion.mode.get()
            self._set_state(
                tcp_position=list(tcp),
                joint_position=list(joints),  # НОВОЕ
                mode=mode

            )
        except Exception as e:
            self._set_state(last_error=str(e))
            self.log.debug(f"Telemetry error: {e}")

    # ---------- Главный цикл ----------
    def run(self) -> None:
        self.log.info("RobotController started")
        while not self.stop_event.is_set():
            try:
                self.check_controller_state()
                self._update_telemetry()
                if self._heartbeat_cb:
                    try:
                        self._heartbeat_cb('rc')
                    except Exception:
                        pass

                try:
                    cmd: Command = self.cmd_queue.get(timeout=0.1)
                except Empty:
                    if not getattr(self, "_nearest_boot_done", False):
                        try:
                            self.find_nearest_waypoint()
                        except Exception as e:
                            self.log.exception(f"Initial nearest calc failed: {e}")
                        finally:
                            self._nearest_boot_done = True
                            self.log.warning(f"_nearest_boot_done {self._nearest_boot_done}")

                    continue

                if cmd.type == CmdType.POWER:
                    self.manipulator_power_control(cmd.payload['state'])
                elif cmd.type == CmdType.FREE_DRIVE:
                    self.manipulator_free_drive(cmd.payload['state'])
                elif cmd.type == CmdType.EXECUTE_TRAJECTORY:
                    self.refresh_waypoints()
                    self.execute_trajectory(RobotTrajectories(cmd.payload['traj']))
                elif cmd.type == CmdType.EXECUTE_ROUTE:
                    self.execute_route(RobotRoutes(cmd.payload['route']))
                elif cmd.type == CmdType.EXECUTE_ACTION:
                    self.execute_action(RobotActions(cmd.payload['action']))
                elif cmd.type == CmdType.MOVE_TO_POINT:
                    self.refresh_waypoints()
                    motion = cmd.payload.get('motion', 'line')
                    self.move_to_point(cmd.payload['name'], motion)
                elif cmd.type == CmdType.IO_SET:
                    self.control_digital_outputs(cmd.payload['index'],
                                                 cmd.payload['value'])
                elif cmd.type == CmdType.REFRESH_WAYPOINTS:
                    self.refresh_waypoints()
                elif cmd.type == CmdType.STOP_MOVE:
                    self.manipulator_stop_drive()
                elif cmd.type == CmdType.FIND_NEAREST:
                    self.find_nearest_waypoint()
                elif cmd.type == CmdType.START_SIMPLE_JOYSTICK:
                    self.start_simple_joystick(cmd.payload.get('coord_sys'))
                elif cmd.type == CmdType.SHUTDOWN:
                    self.log.info("Shutdown command received")
                    break
                else:
                    self.log.warning(f"Unknown command type: {cmd.type}")

            except Exception as e:
                self._set_state(last_error=str(e))
                self.log.exception(f"Unhandled error in RC loop: {e}")

        self.log.info("RobotController stopped")

    # ---------- Завершение ----------
    def stop(self) -> None:
        self.stop_event.set()
