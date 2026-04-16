import enum
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Any, Optional


class CmdType(Enum):
    POWER = auto()  # {'state': False/True}
    FREE_DRIVE = auto()  # {'state': 0|1}
    EXECUTE_TRAJECTORY = auto()  # {'num': int}
    # EXECUTE_ROUTE = auto()  # {'num': int}
    EXECUTE_ACTION = auto()  # {'num': int}
    MOVE_TO_POINT = auto()  # {'name': str}
    GRIPPER_CMD = auto()  # {'index': int, 'value': bool}
    SHIFT_GRIPPER_CMD = auto()  # {'index': int, 'value': bool}
    REFRESH_WAYPOINTS = auto()
    STOP_MOVE = auto()
    SHUTDOWN = auto()
    FIND_NEAREST = auto()
    START_SIMPLE_JOYSTICK = auto()
    WAIT_VTOL_LIFT = auto()  # {'position': 'bottom'|'top', 'timeout_sec': int}


@dataclass
class Command:
    type: CmdType
    payload: Dict[str, Any] | None = None
    source: Optional[str] = None


class RobotPoints(enum.Enum):
    pHomePosition = 1
    pHelicopterModule = 2
    pHelicopter1 = 3
    pHelicopter1Payload = 4
    pHelicopter2 = 5
    pHelicopter2Payload = 6
    pPayload = 7
    pPayload1 = 8
    pPayload2 = 9
    pGrippers = 10
    pGrippers1 = 11
    pGrippers2 = 12
    pCharger = 13
    pCharger1 = 14
    pCharger2 = 15
    pVTOLModule = 16
    pVTOL1 = 17
    pVTOL1Payload = 18
    pVTOL1Battery = 19
    pVTOL2 = 20
    pVTOL2Battery = 21
    pVTOL2Battery2Charge = 22


class RobotTrajectories(enum.Enum):
    tHomePosition_To_HelicopterModule = 1
    tHelicopterModule_To_HomePosition = 2
    tHelicopterModule_To_Helicopter1 = 3
    tHelicopter1_To_HelicopterModule = 4
    tHelicopter1_To_Helicopter1Payload = 5
    tHelicopter1Payload_To_Helicopter1 = 6
    tHelicopterModule_To_Helicopter2 = 7
    tHelicopter2_To_HelicopterModule = 8
    tHelicopter2_To_Helicopter2Payload = 9
    tHelicopter2Payload_To_Helicopter2 = 10
    tHomePosition_To_Payload = 11
    tPayload_To_HomePosition = 12
    tPayload_To_Payload1 = 13
    tPayload1_To_Payload = 14
    tPayload_To_Payload2 = 15
    tPayload2_To_Payload = 16
    tHomePosition_To_Grippers = 17
    tGrippers_To_HomePosition = 18
    tGrippers_To_Grippers1 = 19
    tGrippers1_To_Grippers = 20
    tGrippers_To_Grippers2 = 21
    tGrippers2_To_Grippers = 22
    tHomePosition_To_Charger = 23
    tCharger_To_HomePosition = 24
    tCharger_To_Charger1 = 25
    tCharger1_To_Charger = 26
    tCharger_To_Charger2 = 27
    tCharger2_To_Charger = 28
    tHomePosition_To_VTOLModule = 29
    tVTOLModule_To_HomePosition = 30
    tVTOLModule_To_VTOL1 = 31
    tVTOL1_To_VTOLModule = 32
    tVTOL1_To_VTOL1Payload = 33
    tVTOL1Payload_To_VTOL1 = 34
    tVTOL1_To_VTOL1Battery = 35
    tVTOL1Battery_To_VTOL1 = 36
    tVTOLModule_To_VTOL2 = 37
    tVTOL2_To_VTOLModule = 38
    tVTOL2_To_VTOL2Battery = 39
    tVTOL2Battery_To_VTOL2 = 40
    # Мобильный порт: Легионер наверху, подход сверху с доворотом.
    # Номера 41-42 зарезервированы; конкретные waypoints добавляются при наладке.
    tVTOL2_To_VTOL2Battery_Mobile = 41
    tVTOL2Battery_Mobile_To_VTOL2 = 42


class RobotActions(enum.Enum):
    aVTOL2_To_VTOL2Battery = 1
    aVTOL2Battery_To_VTOL2 = 2
