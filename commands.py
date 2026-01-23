from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Any, Optional

class CmdType(Enum):
    POWER = auto()           # {'state': 1|2}
    FREE_DRIVE = auto()      # {'state': 1|2}
    EXECUTE_ENUM = auto()    # {'cmd': int}
    MOVE_TO_POINT = auto()   # {'name': str}
    IO_SET = auto()          # {'index': int, 'value': bool}
    REFRESH_WAYPOINTS = auto()
    SHUTDOWN = auto()
    FIND_NEAREST = auto()
    START_SIMPLE_JOYSTICK = auto()

@dataclass
class Command:
    type: CmdType
    payload: Dict[str, Any] | None = None
    source: Optional[str] = None