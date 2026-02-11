from PyQt5 import QtCore, QtWidgets
from typing import Dict

from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QMainWindow

from commands import Command, CmdType, RobotTrajectories, RobotRoutes, RobotActions
from config import POINTS_PATH, TRAJ_PATH
from ui_form import Ui_Form
from utils import atomic_write_json


class MainWindow(QMainWindow):
    """
    Класс - основное окно пользователя.
    """

    def __init__(self, robot_controller, cmd_queue, opc_handler):
        super().__init__()
        self.RobotController = robot_controller
        self.cmd_queue = cmd_queue
        self.Waypoints: Dict[str, dict] = {}
        self.Trajectories: Dict[str, dict] = {}
        self.io0_state: bool = False  # 2025_09_29

        self.manipulator_command(
            Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
        self.Waypoints = self.RobotController.get_waypoints_snapshot()
        self.Trajectories = self.RobotController.get_trajectories_snapshot()
        self.Actions = self.RobotController.get_actions_snapshot()
        self.opc_handler = opc_handler

        self.ZGTimer = QtCore.QTimer()
        self.ZGTimer.setInterval(100)  # мс
        self.ZGTimer.timeout.connect(self._zg_tick)

        self.PowerCheckTimer = QtCore.QTimer()
        self.PowerCheckTimer.timeout.connect(self.update_power_button_state)
        self.PowerCheckTimer.start(1000)

        self.InitUI()

    def InitUI(self):
        """Загружаем конфигурацию окна из дизайнера"""
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.ui.ActivateZG.setCheckable(True)
        self.ui.ActivateZG.toggled.connect(self.manipulator_free_drive)
        self.ui.ActivateSJ.clicked.connect(self.start_simple_joystick)
        self.ui.MoveTrajectory.clicked.connect(self.move_by_selected_trajectory)
        self.ui.ExecuteAction.clicked.connect(self.execute_selected_action)
        self.ui.OutputControl.setCheckable(True)
        self.ui.OutputControl.toggled.connect(self.manipulator_gripper_control)
        self.ui.SavePoint.clicked.connect(self.save_current_position)
        self.ui.AddPointToTrajectory.clicked.connect(self.add_current_point_to_trajectory)

        self.ui.comboBox.currentTextChanged.connect(self.waypoint_selected)
        self.ui.TrajectoriesComboBox.currentTextChanged.connect(self.trajectory_selected)
        self.ui.trajListView.clicked.connect(self.on_traj_list_clicked)
        self.ui.actionsListView.clicked.connect(self.on_actions_list_clicked)
        self.update_combo_box()
        self.update_trajectories()
        self.update_actions()
        self.ui.PowerOn.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 1}, source="GUI")
        ))
        self.ui.PowerOff.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 2}, source="GUI")
        ))
        self.ui.MoveToPoint.clicked.connect(self.move_to_selected_point)
        self.ui.StopMove.clicked.connect(self.stop_drive)
        # self.ui.webView.load(QtCore.QUrl("http://192.168.88.100:8080/"))

        self.show()

    def update_power_button_state(self) -> None:
        is_running = (self.RobotController.get_controller_state() == 'run')
        if is_running:
            self.ui.PowerOn.setStyleSheet('background: rgb(124,252,0);')
            self.ui.PowerOff.setStyleSheet('background: rgb(240,240,240);')
        else:
            self.ui.PowerOn.setStyleSheet('background: rgb(240,240,240);')
            self.ui.PowerOff.setStyleSheet('background: rgb(255,0,0);')

    def save_waypoints(self) -> bool:
        try:
            waypoints_list = list(self.Waypoints.values())
            atomic_write_json(POINTS_PATH, {"waypoints": waypoints_list})
            return True
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save waypoints: {e}")
            return False

    def save_trajectories(self) -> bool:
        try:
            trajectories_list = list(self.Trajectories.values())
            atomic_write_json(TRAJ_PATH, {"trajectories": trajectories_list})
            return True
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save trajectories: {e}")
            return False

    def update_combo_box(self) -> None:
        items_model = QStandardItemModel()
        for point in self.Waypoints.keys():
            item = QStandardItem(point)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.comboBox.setModel(items_model)

    def update_trajectories(self) -> None:
        items_model = QStandardItemModel()
        for traj in self.Trajectories.keys():
            item = QStandardItem(traj)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.TrajectoriesComboBox.setModel(items_model)
        self.ui.trajListView.setModel(items_model)

    def update_actions(self) -> None:
        items_model = QStandardItemModel()
        for action in self.Actions.keys():
            item = QStandardItem(action)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.actionsListView.setModel(items_model)

    def on_traj_list_clicked(self) -> None:
        selected = self.ui.trajListView.currentIndex().data()
        self.ui.TrajectoryName.setText(selected)

    def on_actions_list_clicked(self) -> None:
        selected = self.ui.actionsListView.currentIndex().data()
        self.ui.ActionName.setText(selected)

    def trajectory_selected(self, traj_name) -> None:
        self.ui.TrajectoryName.setText(traj_name)

    def waypoint_selected(self, point_name) -> None:
        self.ui.PointName.setText(point_name)
        if point_name in self.Waypoints:
            wp = self.Waypoints[point_name]
            self.ui.SetSpeed.setText(str(wp.get('speed', 0.5)))
            self.ui.SetAccel.setText(str(wp.get('accel', 0.5)))
            self.ui.SetBlend.setText(str(wp.get('blend', 0.0)))

    # def trajectory_selected(self, trajectory_name):
    #     print(trajectory_name)

    def manipulator_command(self, cmd: Command) -> None:
        self.cmd_queue.put(cmd)

    def manipulator_gripper_control(self, clamp: bool) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.IO_SET, {'index': 0, 'value': clamp}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "I/O error",
                                           f"Не удалось изменить состояние DO0: {e}")
            self.ui.OutputControl.setChecked(False)

    def manipulator_free_drive(self, activate: bool) -> None:  # 2025_09_29
        try:
            if activate:
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
                if not self.ZGTimer.isActive():
                    self.ZGTimer.start()
            else:
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            if self.ZGTimer.isActive():
                self.ZGTimer.stop()

    def manipulator_free_drive_old(self, activate: bool) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.FREE_DRIVE, {'state': 1 if activate else 2},
                        source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)

    def add_current_point_to_trajectory(self) -> None:
        point_name = self.ui.PointName.text().strip()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select existing point!")
            return

        trajectory_name = self.ui.TrajectoryName.text().strip()
        if not trajectory_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select trajectory!")
            return

        if point_name in self.Waypoints and trajectory_name in self.Trajectories:
            positions = self.Trajectories[trajectory_name].get('positions')
            positions.append({'name': point_name, 'motion': 'joint'})

            new_trajectory = {
                "name": trajectory_name,
                "positions": positions
            }

            self.Trajectories[trajectory_name] = new_trajectory
            if self.save_trajectories():
                QtWidgets.QMessageBox.information(
                    None,
                    "Success",
                    f"Point '{point_name}' successfully added to trajectory {trajectory_name}!")

    def save_current_position(self) -> None:
        point_name = self.ui.PointName.text().strip()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please enter point name!")
            return

        try:
            tcp_position = self.RobotController.get_current_tcp_position()
            joint_position = self.RobotController.get_current_joint_position()
            speed = float(self.ui.SetSpeed.text())
            accel = float(self.ui.SetAccel.text())
            blend = float(self.ui.SetBlend.text())

            new_point = {
                "name": point_name,
                "speed": speed,
                "accel": accel,
                "blend": blend,
                "tcp": {
                    "x": tcp_position[0],
                    "y": tcp_position[1],
                    "z": tcp_position[2],
                    "Rx": tcp_position[3],
                    "Ry": tcp_position[4],
                    "Rz": tcp_position[5]
                },
                "joints": {
                    "J1": joint_position[0],
                    "J2": joint_position[1],
                    "J3": joint_position[2],
                    "J4": joint_position[3],
                    "J5": joint_position[4],
                    "J6": joint_position[5]
                }
            }

            self.Waypoints[point_name] = new_point

            if self.save_waypoints():
                self.update_combo_box()
                self.update_list_view()
                QtWidgets.QMessageBox.information(None, "Success",
                                                  f"Point '{point_name}' saved successfully!")
        except ValueError:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please enter valid parameters!")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save point: {e}")

    def stop_drive(self) -> None:
        self.manipulator_command(
            Command(CmdType.STOP_MOVE, {}, source="GUI"))

    def move_to_selected_point(self) -> None:
        point_name = self.ui.comboBox.currentText()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a point first!")
            return
        try:
            self.manipulator_command(
                Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
            motion = "line" if self.ui.chkLineMotion.isChecked() else "joint"
            self.manipulator_command(
                Command(CmdType.MOVE_TO_POINT,
                        {'name': point_name, 'motion': motion},
                        source="GUI"))

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Moving to point '{point_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move to point: {e}")

    def move_by_selected_trajectory(self) -> None:
        trajectory_name = self.ui.TrajectoryName.text()
        if not trajectory_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a trajectory first!")
            return

        try:
            cmd_enum = getattr(RobotTrajectories, trajectory_name)
            # команда через OPC
            self.opc_handler.set_command(cmd_enum.value)

            # команда напрямую в манипулятор
            # self.manipulator_command(
            #     Command(CmdType.EXECUTE_TRAJECTORY, {'traj': cmd_enum}, source="GUI"))

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Moving by trajectory '{trajectory_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move by trajectory: {e}")

    def execute_selected_route(self) -> None:
        route_name = self.ui.ActionName.text()
        if not route_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a route first!")
            return
        try:
            # команда через OPC
            route_enum = getattr(RobotRoutes, route_name)
            self.opc_handler.set_action(route_enum.value)

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Executing route '{route_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to execute route: {e}")

    def execute_selected_action(self) -> None:
        action_name = self.ui.ActionName.text()
        if not action_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select an action first!")
            return
        try:
            # команда через OPC
            action_enum = getattr(RobotActions, action_name)
            self.opc_handler.set_action(action_enum.value)

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Executing action '{action_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to execute action: {e}")

    def _zg_tick(self) -> None:  # 2025_09_29
        try:
            if self.ui.ActivateZG.isChecked():
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
            else:
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity periodic toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            self.ZGTimer.stop()

    def start_simple_joystick(self) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.START_SIMPLE_JOYSTICK,
                        {'coord_sys': None}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to start simple joystick: {e}")
