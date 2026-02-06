import enum
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Any, Optional


class CmdType(Enum):
    POWER = auto()  # {'state': 1|2}
    FREE_DRIVE = auto()  # {'state': 1|2}
    EXECUTE_ENUM = auto()  # {'cmd': int}
    EXECUTE_ACTION = auto()  # {'name': str}
    MOVE_TO_POINT = auto()  # {'name': str}
    IO_SET = auto()  # {'index': int, 'value': bool}
    REFRESH_WAYPOINTS = auto()
    STOP_MOVE = auto()
    SHUTDOWN = auto()
    FIND_NEAREST = auto()
    START_SIMPLE_JOYSTICK = auto()


@dataclass
class Command:
    type: CmdType
    payload: Dict[str, Any] | None = None
    source: Optional[str] = None


class RobotCommands(enum.Enum):
    tHomePosition_To_HelicopterModule = 1
    tHelicopterModule_To_HomePosition = 2
    tHelicopterModule_To_Helicopter1 = 3
    tHelicopter1_To_HelicopterModule = 4
    tHelicopter1_To_Helicopter1Load = 5
    tHelicopter1Load_To_Helicopter1 = 6
    tHelicopterModule_To_Helicopter2 = 7
    tHelicopter2_To_HelicopterModule = 8
    tHelicopter2_To_Helicopter2Load = 9
    tHelicopter2Load_To_Helicopter2 = 10
    tHomePosition_To_Load = 11
    tLoad_To_HomePosition = 12
    tLoad_To_Load1 = 13
    tLoad1_To_Load = 14
    tLoad_To_Load2 = 15
    tLoad2_To_Load = 16
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
    tVTOL1_To_VTOL1Load = 33
    tVTOL1Load_To_VTOL1 = 34
    tVTOL1_To_VTOL1Battery = 35
    tVTOL1Battery_To_VTOL1 = 36
    tVTOLModule_To_VTOL2 = 37
    tVTOL2_To_VTOLModule = 38
    tVTOL2_To_VTOL2Battery = 39
    tVTOL2Battery_To_VTOL2 = 40
