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

OPC_CLIENT_TIME = 1

ENABLE_MOVE_TO_MODULE_H = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToModuleH"
ENABLE_MOVE_TO_MODULE_V = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToModuleV"
ENABLE_MOVE_TO_CHARGER_H = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToChargerH"
ENABLE_MOVE_TO_CHARGER_V = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToChargerV"
ENABLE_MOVE_TO_PAYLOAD_STORAGE = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToPayloadStorage"
ENABLE_MOVE_TO_GRIPPERS_STORAGE = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToGrippersStorage"
ENABLE_MOVE_TO_HOME = "|var|HCFA-PLC.Application.OPC.Robot.qxRobotEnableMoveToHome"

HELICOPTER_MODULE = ["pHelicopter1", "pHelicopter2", "pHomePosition"]
VTOL_MODULE = ["pVTOL1", "pVTOL2", "pHomePosition"]
LOAD_STORAGE = ["pLoad1", "pLoad2", "pHomePosition"]
GRIPPERS_STORAGE = ["pHomePosition"]
CHARGER_H = ["pHomePosition"]
CHARGER_V = ["pHomePosition"]
HOME_POSITION = ["pHelicopterModule", "pVTOLModule", "pLoad", "pGrippers", "pCharger"]

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

H_TABLE_HATCH_OPENED = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixOpened"
H_TABLE_HATCH_CLOSED = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixClosed"
H_TABLE_HATCH_ALARM = "|var|HCFA-PLC.Application.OPC.H_Table.Hatch.ixAlarm"

V_TABLE_HATCH_OPENED = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixOpened"
V_TABLE_HATCH_CLOSED = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixClosed"
V_TABLE_HATCH_ALARM = "|var|HCFA-PLC.Application.OPC.V_Table.Hatch.ixAlarm"
