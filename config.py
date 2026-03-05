from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# Пути к данным
POINTS_PATH = BASE_DIR / "points.json"
TRAJ_PATH = BASE_DIR / "trajectories.json"

# Параметры подключения
ROBOT_IP = "127.0.0.1"
# OPC_ENDPOINT = "opc.tcp://0.0.0.0:4840"  # для отладки по месту
OPC_ENDPOINT = "opc.tcp://127.0.0.1:4840"  # для теста
PLC_MANIPULATOR_ADDRESS = "opc.tcp://192.168.88.100:4840/freeopcua/server/"  # для теста
PLC_VT_ADDRESS = "opc.tcp://192.168.88.101:4840/freeopcua/server/"  # для теста
PLC_VTOL_ADDRESS = "opc.tcp://192.168.88.102:4840/freeopcua/server/"  # для теста

# Команды
EXEC_TRAJ = "EXECUTE_TRAJECTORY"
IO_SET = "IO_SET"

# IO и прочее
NUM_DIGITAL_IO = 24
GRIPPER_DO_INDEX = 0
SHIFT_GRIPPER_DO_INDEX = 1
LOG_PATH = BASE_DIR / "robopro.log"

"""Состояния выполнения команды"""
EXECUTION = 100
FINISHED = 200
EXCEPTION = 300
BLOCK = 400

# OPC
OPC_CLIENT_TIME = 1

# PLC command nodes (читаются OPC клиентом, пересылаются в cmd_queue)
PLC_CMD_POWER_ON = "|var|HCFA-PLC.Application.OPC.Robot.qxPowerOn"
PLC_CMD_FREE_DRIVE = "|var|HCFA-PLC.Application.OPC.Robot.qxFreeDrive"
PLC_CMD_FIND_NEAREST = "|var|HCFA-PLC.Application.OPC.Robot.qxFindNearest"
PLC_CMD_GRIPPER = "|var|HCFA-PLC.Application.OPC.Robot.qxGripperCmd"
PLC_CMD_SHIFT_GRIPPER = "|var|HCFA-PLC.Application.OPC.Robot.qxShiftGripper"
PLC_CMD_ACTION = "|var|HCFA-PLC.Application.OPC.Robot.qxAction"
PLC_CMD_TRAJECTORY = "|var|HCFA-PLC.Application.OPC.Robot.qxTrajectory"

ENABLE_MOVE_TO_MODULE_H = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToModuleH"
ENABLE_MOVE_TO_MODULE_V = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToModuleV"
ENABLE_MOVE_TO_CHARGER_H = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToChargerH"
ENABLE_MOVE_TO_CHARGER_V = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToChargerV"
ENABLE_MOVE_TO_PAYLOAD_STORAGE = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToPayloadStorage"
ENABLE_MOVE_TO_GRIPPERS_STORAGE = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToGrippersStorage"
ENABLE_MOVE_TO_HOME = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToHome"

HELICOPTER_MODULE = ["pHelicopter1", "pHelicopter2", "pHomePosition"]
VTOL_MODULE = ["pVTOL1", "pVTOL2", "pHomePosition"]
LOAD_STORAGE = ["pPayload1", "pPayload2", "pHomePosition"]
GRIPPERS_STORAGE = ["pHomePosition"]
CHARGER_H = ["pHomePosition"]
CHARGER_V = ["pHomePosition"]
HOME_POSITION = ["pHelicopterModule", "pVTOLModule", "pPayload", "pGrippers", "pCharger"]

X_POSITION_MODULE_H = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionModuleH"
X_POSITION_MODULE_V = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionModuleV"
X_POSITION_CHARGER_H = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionChargerH"
X_POSITION_CHARGER_V = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionChargerV"
X_POSITION_LOAD = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionPayload"
X_POSITION_GRIPPER_STORAGE = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPositionGripperStorage"
X_POSITION_HAS_ZEROED = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixHasZeroed"
X_POSITION_POWERED = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixPowered"
X_POSITION_ALARM = "|var|HCFA-PLC.Application.OPC.M_Table.X.ixAlarm"

Y_POSITION_MODULE_H = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionModuleH"
Y_POSITION_MODULE_V = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionModuleV"
Y_POSITION_CHARGER_H = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionChargerH"
Y_POSITION_CHARGER_V = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionChargerV"
Y_POSITION_LOAD = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionPayload"
Y_POSITION_GRIPPER_STORAGE = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPositionGripperStorage"
Y_POSITION_HAS_ZEROED = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixHasZeroed"
Y_POSITION_POWERED = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixPowered"
Y_POSITION_ALARM = "|var|HCFA-PLC.Application.OPC.M_Table.Y.ixAlarm"

# Watchdog узлы (ping/pong между клиентом и PLC)
OPC_WATCHDOG_WRITE_NODE = "|var|HCFA-PLC.Application.OPC.Watchdog.CheckOut"
OPC_WATCHDOG_READ_NODE = "|var|HCFA-PLC.Application.OPC.Watchdog.CheckIn"

H_TABLE_HATCH_OPENED = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixOpened"
H_TABLE_HATCH_CLOSED = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixClosed"
H_TABLE_HATCH_ALARM = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixAlarm"
H_TABLE_LIFT_POS_TOP = "|var|HCFA-PLC.Application.OPC.H_Table.Lift.ixPositionTop"
H_TABLE_LIFT_POS_BOTTOM = "|var|HCFA-PLC.Application.OPC.H_Table.Lift.ixPositionBottom"
H_TABLE_LIFT_ALARM = "|var|HCFA-PLC.Application.OPC.H_Table.Lift.ixAlarm"

H_BOX_LIFT_POS_TOP = "|var|HCFA-PLC.Application.OPC.H_Box.PlugChargerLift.ixTop"
H_BOX_LIFT_POS_BOTTOM = "|var|HCFA-PLC.Application.OPC.H_Box.PlugChargerLift.ixBottom"
H_BOX_LIFT_ALARM = "|var|HCFA-PLC.Application.OPC.H_Box.PlugChargerLift.ixAlarm"

V_TABLE_HATCH_OPENED = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixOpened"
V_TABLE_HATCH_CLOSED = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixClosed"
V_TABLE_HATCH_ALARM = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixAlarm"
V_TABLE_LIFT_POS_TOP = "|var|HCFA-PLC.Application.OPC.V_Table.Lift.ixPositionTop"
V_TABLE_LIFT_POS_BOTTOM = "|var|HCFA-PLC.Application.OPC.V_Table.Lift.ixPositionBottom"
V_TABLE_LIFT_ALARM = "|var|HCFA-PLC.Application.OPC.V_Table.Lift.ixAlarm"

# стили для trajectory_mao
RED_COLOR = "background:#fce4ec; border-radius:6px; padding:6px; color:#b71c1c;"
GREEN_COLOR = "background:#e8f5e9; border-radius:6px; padding:6px; color:#2e7d32; font-weight:bold;"
BEIGE_COLOR = "background:#fff3e0; border-radius:6px; padding:6px; color:#e65100; font-weight:bold;"
BLUE_COLOR = "background:#e3f2fd; border-radius:6px; padding:6px; color:#1565c0;"
ALABASTER_COLOR = "border:none; background:#fafafa;"
