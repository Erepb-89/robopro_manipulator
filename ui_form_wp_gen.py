from PyQt5 import QtCore, QtWidgets
from typing import Dict

from PyQt5.QtWidgets import QMainWindow

from commands import Command, CmdType
from config import POINTS_PATH
from robot_controller import trajectories_dict
from ui_form import Ui_Form
from utils import atomic_write_json


class MainWindow(QMainWindow):
    """
    Класс - основное окно пользователя.
    """

    def __init__(self, robot_controller, cmd_queue):
        super().__init__()
        self.RobotController = robot_controller
        self.cmd_queue = cmd_queue
        self.WaypointsFile = str(POINTS_PATH)
        self.Waypoints: Dict[str, dict] = {}
        self.Trajectories: Dict[str, dict] = {}
        self.io0_state: bool = False  # 2025_09_29

        self.manupilator_command(
            Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
        self.Waypoints = self.RobotController.get_waypoints_snapshot()
        self.Trajectories = self.RobotController.get_trajectories_snapshot()

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

        self.ui.ActivateZG.toggled.connect(self.manipulator_free_drive)
        self.ui.ActivateSJ.clicked.connect(self.start_simple_joystick)
        self.ui.MoveTrajectory.clicked.connect(self.move_by_selected_trajectory)
        self.ui.OutputControl.toggled.connect(self.manipulator_gripper_control)
        self.ui.SavePoint.clicked.connect(self.save_current_position)
        self.ui.comboBox.currentTextChanged.connect(self.waypoint_selected)
        self.update_combo_box()
        self.ui.TrajectoriesComboBox.currentTextChanged.connect(self.trajectory_selected)
        self.update_trajectories_combo_box()
        self.ui.PowerOn.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 1}, source="GUI")
        ))
        self.ui.PowerOff.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 2}, source="GUI")
        ))
        self.ui.MoveToPoint.clicked.connect(self.move_to_selected_point)

        self.show()

    # class Ui_Form(object):
    #
    #     def __init__(self, robot_controller, cmd_queue):
    #         self.RobotController = robot_controller
    #         self.cmd_queue = cmd_queue
    #         self.WaypointsFile = str(POINTS_PATH)
    #         self.Waypoints: Dict[str, dict] = {}
    #         self.Trajectories: Dict[str, dict] = {}
    #         self.io0_state: bool = False #2025_09_29
    #
    #     def setupUi(self, Form):
    #         Form.setObjectName("Form")
    #         Form.resize(570, 310)
    #
    #         self.manupilator_command(
    #             Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
    #         self.Waypoints = self.RobotController.get_waypoints_snapshot()
    #         self.Trajectories = self.RobotController.get_trajectories_snapshot()
    #
    #         self.ActivateZG = QtWidgets.QPushButton(Form)
    #         self.ActivateZG.setGeometry(QtCore.QRect(300, 30, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.ActivateZG.setFont(font)
    #         self.ActivateZG.setObjectName("ActivateZG")
    #         self.ActivateZG.setCheckable(True)
    #         self.ActivateZG.toggled.connect(self.manipulator_free_drive)
    #         self.ZGTimer = QtCore.QTimer()
    #         self.ZGTimer.setInterval(100)  # мс
    #         self.ZGTimer.timeout.connect(self._zg_tick)
    #
    #         self.ActivateSJ = QtWidgets.QPushButton(Form)
    #         self.ActivateSJ.setGeometry(QtCore.QRect(440, 30, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.ActivateSJ.setFont(font)
    #         self.ActivateSJ.setObjectName("SimpleJoystick")
    #         self.ActivateSJ.clicked.connect(self.start_simple_joystick)
    #
    #         self.MoveTrajectory = QtWidgets.QPushButton(Form)
    #         self.MoveTrajectory.setGeometry(QtCore.QRect(440, 90, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.MoveTrajectory.setFont(font)
    #         self.MoveTrajectory.setObjectName("MoveTrajectory")
    #         self.MoveTrajectory.clicked.connect(self.move_by_selected_trajectory)
    #
    #         self.OutputControl = QtWidgets.QPushButton(Form)
    #         self.OutputControl.setGeometry(QtCore.QRect(160, 90, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.OutputControl.setFont(font)
    #         self.OutputControl.setObjectName("DO Gripper")
    #         self.OutputControl.setCheckable(True)
    #         self.OutputControl.toggled.connect(self.manipulator_gripper_control)
    #
    #         self.chkLineMotion = QtWidgets.QCheckBox(Form)
    #         self.chkLineMotion.setGeometry(QtCore.QRect(30, 5, 150, 24))
    #         self.chkLineMotion.setText("Line motion")
    #         self.chkLineMotion.setChecked(False)
    #         self.chkLineMotion.setToolTip("Если включено — движение по TCP (line). "
    #                                       "Если выключено — суставное(joint).")
    #
    #         self.PointNameSelectText = QtWidgets.QLabel(Form)
    #         self.PointNameSelectText.setGeometry(QtCore.QRect(270, 150, 191, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.PointNameSelectText.setFont(font)
    #         self.PointNameSelectText.setObjectName("PointNameSelectText")
    #
    #         self.TrajectoryNameSelectText = QtWidgets.QLabel(Form)
    #         self.TrajectoryNameSelectText.setGeometry(QtCore.QRect(270, 210, 191, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.TrajectoryNameSelectText.setFont(font)
    #         self.TrajectoryNameSelectText.setObjectName("TrajectoryNameSelectText")
    #
    #         self.SavePoint = QtWidgets.QPushButton(Form)
    #         self.SavePoint.setGeometry(QtCore.QRect(20, 90, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.SavePoint.setFont(font)
    #         self.SavePoint.setObjectName("SavePoint")
    #         self.SavePoint.clicked.connect(self.save_current_position)
    #
    #         self.PointName = QtWidgets.QLineEdit(Form)
    #         self.PointName.setGeometry(QtCore.QRect(20, 170, 200, 30))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.PointName.setFont(font)
    #         self.PointName.setObjectName("PointName")
    #
    #         self.SetSpeed = QtWidgets.QLineEdit(Form)
    #         self.SetSpeed.setGeometry(QtCore.QRect(80, 205, 75, 30))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.SetSpeed.setFont(font)
    #         self.SetSpeed.setObjectName("SetSpeed")
    #
    #         self.SetAccel = QtWidgets.QLineEdit(Form)
    #         self.SetAccel.setGeometry(QtCore.QRect(80, 240, 75, 30))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.SetAccel.setFont(font)
    #         self.SetAccel.setObjectName("SetAccel")
    #
    #         self.SetBlend = QtWidgets.QLineEdit(Form)
    #         self.SetBlend.setGeometry(QtCore.QRect(80, 275, 75, 30))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.SetBlend.setFont(font)
    #         self.SetBlend.setObjectName("SetBlend")
    #
    #         self.comboBox = QtWidgets.QComboBox(Form)
    #         self.comboBox.setGeometry(QtCore.QRect(270, 170, 200, 30))
    #         self.comboBox.setObjectName("comboBox")
    #         self.comboBox.currentTextChanged.connect(self.waypoint_selected)
    #         self.update_combo_box()
    #
    #         self.TrajectoriesComboBox = QtWidgets.QComboBox(Form)
    #         self.TrajectoriesComboBox.setGeometry(QtCore.QRect(270, 230, 200, 30))
    #         self.TrajectoriesComboBox.setObjectName("TrajectoriesComboBox")
    #         self.TrajectoriesComboBox.currentTextChanged.connect(self.trajectory_selected)
    #         self.update_trajectories_combo_box()
    #
    #         self.WaypointName = QtWidgets.QLabel(Form)
    #         self.WaypointName.setGeometry(QtCore.QRect(20, 150, 151, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.WaypointName.setFont(font)
    #         self.WaypointName.setObjectName("WaypointName")
    #
    #         self.PowerOn = QtWidgets.QPushButton(Form)
    #         self.PowerOn.setGeometry(QtCore.QRect(20, 30, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.PowerOn.setFont(font)
    #         self.PowerOn.setObjectName("PowerOn")
    #         self.PowerOn.clicked.connect(lambda: self.cmd_queue.put(
    #             Command(CmdType.POWER, {'state': 1}, source="GUI")
    #         ))
    #
    #         self.PowerOff = QtWidgets.QPushButton(Form)
    #         self.PowerOff.setGeometry(QtCore.QRect(160, 30, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.PowerOff.setFont(font)
    #         self.PowerOff.setObjectName("PowerOff")
    #         self.PowerOff.clicked.connect(lambda: self.cmd_queue.put(
    #             Command(CmdType.POWER, {'state': 2}, source="GUI")
    #         ))
    #
    #         self.Speed = QtWidgets.QLabel(Form)
    #         self.Speed.setGeometry(QtCore.QRect(20, 210, 75, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.Speed.setFont(font)
    #         self.Speed.setObjectName("Speed")
    #
    #         self.Accel = QtWidgets.QLabel(Form)
    #         self.Accel.setGeometry(QtCore.QRect(20, 245, 75, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.Accel.setFont(font)
    #         self.Accel.setObjectName("Accel")
    #
    #         self.Blend = QtWidgets.QLabel(Form)
    #         self.Blend.setGeometry(QtCore.QRect(20, 280, 75, 20))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.Blend.setFont(font)
    #         self.Blend.setObjectName("Blend")
    #
    #         self.MoveToPoint = QtWidgets.QPushButton(Form)
    #         self.MoveToPoint.setGeometry(QtCore.QRect(300, 90, 120, 50))
    #         font = QtGui.QFont()
    #         font.setPointSize(12)
    #         self.MoveToPoint.setFont(font)
    #         self.MoveToPoint.setObjectName("MoveToPoint")
    #         self.MoveToPoint.clicked.connect(self.move_to_selected_point)
    #
    #         self.PowerCheckTimer = QtCore.QTimer()
    #         self.PowerCheckTimer.timeout.connect(self.update_power_button_state)
    #         self.PowerCheckTimer.start(1000)
    #
    #         self.retranslateUi(Form)
    #         QtCore.QMetaObject.connectSlotsByName(Form)
    #
    #     def retranslateUi(self, Form):
    #         _ = QtCore.QCoreApplication.translate
    #         Form.setWindowTitle(_("Form", "Waypoint Generate Panel"))
    #         self.ActivateZG.setText(_("Form", "Activate ZG"))
    #         self.ActivateSJ.setText(_("Form", "SimpleJoystick"))
    #         self.OutputControl.setText(_("Form", "DO Gripper"))
    #         self.PointNameSelectText.setText(_("Form", "Waypoints list:"))
    #         self.TrajectoryNameSelectText.setText(_("Form", "Trajectories list:"))
    #         self.SavePoint.setText(_("Form", "SavePoint"))
    #         self.WaypointName.setText(_("Form", "Waypoint Name:"))
    #         self.PowerOn.setText(_("Form", "Power On"))
    #         self.PowerOff.setText(_("Form", "Power OFF"))
    #         self.Speed.setText(_("Form", "Speed"))
    #         self.Accel.setText(_("Form", "Accel"))
    #         self.Blend.setText(_("Form", "Blend"))
    #         self.MoveToPoint.setText(_("Form", "MoveToPoint"))
    #         self.MoveTrajectory.setText(_("Form", "Move By Trajectory"))

    def update_power_button_state(self):
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

    def update_combo_box(self):
        self.ui.comboBox.clear()
        for point_name in self.Waypoints.keys():
            self.ui.comboBox.addItem(point_name)

    def update_trajectories_combo_box(self):
        self.ui.TrajectoriesComboBox.clear()
        for trajectory_name in self.Trajectories.keys():
            self.ui.TrajectoriesComboBox.addItem(trajectory_name)

    def waypoint_selected(self, point_name):
        self.ui.PointName.setText(point_name)
        if point_name in self.Waypoints:
            wp = self.Waypoints[point_name]
            self.ui.SetSpeed.setText(str(wp.get('speed', 0.5)))
            self.ui.SetAccel.setText(str(wp.get('accel', 0.5)))
            self.ui.SetBlend.setText(str(wp.get('blend', 0.0)))

    def trajectory_selected(self, trajectory_name):
        print(trajectory_name)

    def manupilator_command(self, cmd: Command):
        self.cmd_queue.put(cmd)

    def manipulator_gripper_control(self, clamp: bool):
        try:
            self.manupilator_command(
                Command(CmdType.IO_SET, {'index': 0, 'value': clamp}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "I/O error",
                                           f"Не удалось изменить состояние DO0: {e}")
            self.ui.OutputControl.setChecked(False)

    def manipulator_free_drive(self, activate: bool):  # 2025_09_29
        try:
            if activate:
                self.manupilator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
                if not self.ZGTimer.isActive():
                    self.ZGTimer.start()
            else:
                self.manupilator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            if self.ZGTimer.isActive():
                self.ZGTimer.stop()

    def manipulator_free_drive_old(self, activate: bool):
        try:
            self.manupilator_command(
                Command(CmdType.FREE_DRIVE, {'state': 1 if activate else 2},
                        source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)

    def save_current_position(self):
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
                QtWidgets.QMessageBox.information(None, "Success",
                                                  f"Point '{point_name}' saved successfully!")
        except ValueError:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please enter valid parameters!")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save point: {e}")

    def move_to_selected_point(self):
        point_name = self.ui.comboBox.currentText()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a point first!")
            return
        try:
            self.manupilator_command(
                Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
            motion = "line" if self.ui.chkLineMotion.isChecked() else "joint"
            self.manupilator_command(
                Command(CmdType.MOVE_TO_POINT,
                        {'name': point_name, 'motion': motion},
                        source="GUI"))

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Moving to point '{point_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move to point: {e}")

    def move_by_selected_trajectory(self):
        trajectory_name = self.ui.comboBox.currentText()
        if not trajectory_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a point first!")
            return
        try:
            self.manupilator_command(
                Command(CmdType.EXECUTE_ENUM, {'cmd': 3}, source="GUI"))

            QtWidgets.QMessageBox.information(None, "Success",
                                              f"Moving by trajectory '{trajectory_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move by trajectory: {e}")

    def _zg_tick(self):  # 2025_09_29
        try:
            if self.ui.ActivateZG.isChecked():
                self.manupilator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
            else:
                self.manupilator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity periodic toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            self.ZGTimer.stop()

    def start_simple_joystick(self):
        try:
            self.manupilator_command(
                Command(CmdType.START_SIMPLE_JOYSTICK,
                        {'coord_sys': None}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to start simple joystick: {e}")
