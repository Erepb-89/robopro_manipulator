from opcua import Server, Client
import time
import threading
from queue import Queue
from commands import Command, CmdType
from config import GRIPPER_DO_INDEX
from typing import Optional


class OPCHandler:

    def __init__(self, url: str, robot_controller, cmd_queue: Queue, logger,
                 heartbeat_cb=None):
        self.Url = url
        self.RobotController = robot_controller
        self.cmd_queue = cmd_queue
        self.log = logger

        self.Server = Server()
        self.Server.set_endpoint(self.Url)

        self.AERI = self.Server.get_objects_node()
        self.ns = self.Server.register_namespace("AERI_ROBOT_NS")
        self.Manipulator = self.AERI.add_object(self.ns, "Manipulator")

        self.Client = Client("opc.tcp://0.0.0.0:4840")

        self.TcpPosition = self.Manipulator.add_variable(self.ns, "TcpPosition",
                                                         [0.0] * 6)
        # self.Speed = self.Manipulator.add_variable(self.ns, "Speed",
        #                                           0.0)  # для будущего
        # self.Accel = self.Manipulator.add_variable(self.ns, "Accel", 0.0)
        self.qTrajectory = self.Manipulator.add_variable(self.ns, "qTrajectory",
                                                         0)  # EXECUTE_TRAJECTORY
        self.qTrajectory.set_writable()

        self.qAction = self.Manipulator.add_variable(self.ns, "qAction",
                                                     0)  # EXECUTE_ACTION
        self.qAction.set_writable()

        self.qRoute = self.Manipulator.add_variable(self.ns, "qRoute",
                                                    0)  # EXECUTE_ROUTE
        self.qRoute.set_writable()

        self.qPowerOn = self.Manipulator.add_variable(self.ns, "qPowerOn",
                                                      0)  # 1/2
        self.qPowerOn.set_writable()

        self.qFreeDrive = self.Manipulator.add_variable(self.ns, "qFreeDrive",
                                                        0)  # 1/2
        self.qFreeDrive.set_writable()

        self.qGripperCmd = self.Manipulator.add_variable(self.ns,
                                                         "qGripperCmd", 0)
        self.qGripperCmd.set_writable()

        self.qFindNearest = self.Manipulator.add_variable(self.ns,
                                                          "qFindNearest", 0)
        self.qFindNearest.set_writable()

        self.NearestWaypoint = self.Manipulator.add_variable(self.ns,
                                                             "NearestWaypoint", "")

        self.NearestTrajList = self.Manipulator.add_variable(self.ns,
                                                             "NearestTrajList", "")

        self.NearestDist = self.Manipulator.add_variable(self.ns,
                                                         "NearestDist", "")

        self.ControllerState = self.Manipulator.add_variable(self.ns,
                                                             "ControllerState", "")
        self.SafetyStatus = self.Manipulator.add_variable(self.ns,
                                                          "SafetyStatus", "")
        self.Mode = self.Manipulator.add_variable(self.ns, "Mode", "")
        self.LastError = self.Manipulator.add_variable(self.ns, "LastError", "")

        self.CmdState = self.Manipulator.add_variable(self.ns, "CmdState", 0)

        self.running = False
        self.stop_event = threading.Event()

        self._heartbeat_cb = heartbeat_cb

        self._last_nearest_wp: Optional[str] = None
        self._last_nearest_traj: Optional[str] = None
        self._last_nearest_dist: Optional[float] = None
        self._last_cmd_state: Optional[int] = 0

    def update_nearest_info(self) -> None:
        try:
            trigger = int(self.qFindNearest.get_value())
            if trigger == 1:
                self.cmd_queue.put(Command(CmdType.FIND_NEAREST, {},
                                           source="OPC"))
                self.qFindNearest.set_value(0)

            info = self.RobotController.get_nearest_info()
            if not info:
                return

            wp = info.get("waypoint", "")
            traj = ",".join(info.get("trajectories", []))
            dist = float(info.get("distance", 0.0))

            if self._last_nearest_wp != wp:
                self.NearestWaypoint.set_value(wp)
                self._last_nearest_wp = wp

            if self._last_nearest_traj != traj:
                self.NearestTrajList.set_value(traj)
                self._last_nearest_traj = traj

            if self._last_nearest_dist != dist:
                self.NearestDist.set_value(dist)
                self._last_nearest_dist = dist
        except Exception as e:
            raise RuntimeError(f"update nearest info Error")

    def handle_gripper_cmd(self) -> None:  # 1 = ON, 2 = OFF
        try:
            gcmd = int(self.qGripperCmd.get_value())
            if gcmd == 1:
                self.cmd_queue.put(Command(CmdType.IO_SET,
                                           {'index': GRIPPER_DO_INDEX,
                                            'value': True},
                                           source="OPC"))
                self.qGripperCmd.set_value(0)

            if gcmd == 2:
                self.cmd_queue.put(Command(CmdType.IO_SET,
                                           {'index': GRIPPER_DO_INDEX,
                                            'value': False},
                                           source="OPC"))
                self.qGripperCmd.set_value(0)
        except Exception as e:
            raise RuntimeError(f"qGripperCmd Error")

    def set_command(self, value):
        self.qTrajectory.set_value(value)

    def set_route(self, value):
        self.qRoute.set_value(value)

    def set_action(self, value):
        self.qAction.set_value(value)

    def start(self) -> None:
        qCmdPrev = 0
        self.running = True
        self.Server.start()
        self.log.info(f"OPC UA server running at {self.Url}")

        try:
            while not self.stop_event.is_set():
                try:
                    st = self.RobotController.get_state_snapshot()
                    self.TcpPosition.set_value(st.tcp_position)
                    self.ControllerState.set_value(st.controller_state or "")
                    self.SafetyStatus.set_value(st.safety_status or "")
                    self.Mode.set_value(st.mode or "")
                    self.LastError.set_value(st.last_error or "")

                    qPowerOn = self.qPowerOn.get_value()
                    qCmd = self.qTrajectory.get_value()
                    qFreeDrive = self.qFreeDrive.get_value()

                    if qPowerOn != 0:
                        self.cmd_queue.put(
                            Command(CmdType.POWER, {'state': int(qPowerOn)},
                                    source="OPC"))
                        self.qPowerOn.set_value(0)

                    if qCmdPrev != qCmd:
                        if qCmd == 0:
                            self.cmd_queue.put(
                                Command(CmdType.STOP_MOVE, {},
                                        source="OPC"))
                        else:
                            if qCmd in range(1, 100):
                                self.cmd_queue.put(
                                    Command(CmdType.EXECUTE_TRAJECTORY, {'traj': int(qCmd)},
                                            source="OPC"))
                    qCmdPrev = qCmd

                    if qFreeDrive != 0:
                        self.cmd_queue.put(Command(CmdType.FREE_DRIVE,
                                                   {'state': int(qFreeDrive)},
                                                   source="OPC"))
                        self.qFreeDrive.set_value(0)

                    self.handle_gripper_cmd()
                    self.update_nearest_info()

                    if self._last_cmd_state != st.cmd_state:
                        self.CmdState.set_value(st.cmd_state)
                        self._last_cmd_state = st.cmd_state

                except Exception as e:
                    self.log.error(f"Error in OPC loop: {e}")

                if self._heartbeat_cb:
                    try:
                        self._heartbeat_cb('opc')
                    except Exception:
                        pass

                time.sleep(0.2)

        finally:
            try:
                self.Server.stop()
            except Exception as e:
                self.log.error(f"OPC stop error: {e}")
            self.log.info("OPC UA server stopped.")

    def stop(self) -> None:
        self.stop_event.set()
