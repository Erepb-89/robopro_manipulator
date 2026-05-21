import enum
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Any, Optional


class CmdType(Enum):
    POWER = auto()  # {'state': False/True}
    FREE_DRIVE = auto()  # {'state': 0|1}
    EXECUTE_TRAJECTORY = auto()  # {'num': int}
    EXECUTE_ROUTE = auto()  # {'num': int}
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
    pUndefined = 0
    pHomePosition = 100
    pHelicopterModule = 200
    pHelicopter2 = 300
    pHelicopter2Payload = 400
    pHelicopter2Battery1 = 500
    pHelicopter2InsideBattery1Slot = 600
    pHelicopter2BeforeBattery1Slot = 700
    pHelicopter2Battery2 = 800
    pHelicopter2InsideBattery2Slot = 900
    pHelicopter2BeforeBattery2Slot = 1000
    pHelicopter1 = 1100
    pPayload = 1200
    pPayload1 = 1300
    pPayload1InsideSlot = 1400
    pPayload1BeforeSlot = 1500
    pPayload2 = 1600
    pPayload2InsideSlot = 1700
    pPayload2BeforeSlot = 1800
    pGrippers = 1900
    pGrippers1 = 2000
    pGrippers1InsideSlot = 2100
    pGrippers1BeforeSlot = 2200
    pGrippers2 = 2300
    pGrippers2InsideSlot = 2400
    pGrippers2BeforeSlot = 2500
    pCharger = 2600
    pHelicopter2Charger1 = 2700
    pHelicopter2Charger1Slot1 = 2800
    pHelicopter2Charger1InsideSlot1 = 2900
    pHelicopter2Charger1BeforeSlot1 = 3000
    pHelicopter2Charger1Slot2 = 3100
    pHelicopter2Charger1InsideSlot2 = 3200
    pHelicopter2Charger1BeforeSlot2 = 3300
    pVTOL2Charger1 = 3400
    pVTOL2Charger1Slot1 = 3500
    pVTOL2Charger1InsideSlot1 = 3600
    pVTOL2Charger1BeforeSlot1 = 3700
    pVTOL2Charger1Slot2 = 3800
    pVTOL2Charger1InsideSlot2 = 3900
    pVTOL2Charger1BeforeSlot2 = 4000
    pVTOL2Charger2 = 4100
    pVTOL2Charger2Slot1 = 4200
    pVTOL2Charger2InsideSlot1 = 4300
    pVTOL2Charger2BeforeSlot1 = 4400
    pVTOL2Charger2Slot2 = 4500
    pVTOL2Charger2InsideSlot2 = 4600
    pVTOL2Charger2BeforeSlot2 = 4700
    pVTOLModule = 4800
    pVTOL1 = 4900
    pVTOL1Payload = 5000
    pVTOL1InsidePayloadSlot = 5100
    pVTOL1BeforePayloadSlot = 5200
    pVTOL2 = 5300
    pVTOL2Battery1 = 5400
    pVTOL2InsideBattery1Slot = 5500
    pVTOL2BeforeBattery1Slot = 5600
    pVTOL2Battery2 = 5700
    pVTOL2InsideBattery2Slot = 5800
    pVTOL2BeforeBattery2Slot = 5900

    # Вспомогательные точки (не отображаются на карте, но нужны для траекторий)
    pVTOL2Charger1BeforeSlot1_pre = 99000
    pHelicopter2Charger1_bef = 99001
    pHelicopter2Charger1BeforeSlot1_2 = 99002
    pVTOL2Charger1_mid = 99003
    pVTOL2Charger1InSlot1_pre = 99004
    pVTOL2Charger1InSlot1_pre2 = 99005
    pVTOL2_under_cap = 99006
    pVTOL2_open_cap1 = 99007
    pVTOL2_open_cap2 = 99008
    pVTOL2_open_cap3 = 99009
    pVTOL2_open_cap4 = 99010
    pVTOL2_open_cap5 = 99011
    pVTOL2_open_cap6 = 99012
    pVTOL2_open_cap7 = 99013
    pVTOL2_open_cap8 = 99014
    pVTOL2_open_cap9 = 99015
    pVTOL2_open_cap10 = 99016
    pVTOL2_open_cap11 = 99017
    pVTOL2_open_cap12 = 99018
    pVTOL2_open_cap13 = 99019
    pVTOL2_open_cap14 = 99020
    pVTOL2_open_cap15 = 99021


class RobotTrajectories(enum.Enum):
    tHomePosition_To_HelicopterModule = 1
    tHelicopterModule_To_HomePosition = 2
    tHelicopterModule_To_Helicopter1 = 3
    tHelicopter1_To_HelicopterModule = 4
    tHelicopterModule_To_Helicopter2 = 5
    tHelicopter2_To_HelicopterModule = 6
    tHelicopter2_To_Helicopter2Payload = 7
    tHelicopter2Payload_To_Helicopter2 = 8
    tHomePosition_To_Payload = 9
    tPayload_To_HomePosition = 10
    tPayload_To_Payload1 = 11
    tPayload1_To_Payload = 12
    tPayload_To_Payload2 = 13
    tPayload2_To_Payload = 14
    tHomePosition_To_Grippers = 15
    tGrippers_To_HomePosition = 16
    tGrippers_To_Grippers1 = 17
    tGrippers1_To_Grippers = 18
    tGrippers_To_Grippers2 = 19
    tGrippers2_To_Grippers = 20
    tHomePosition_To_Charger = 21
    tCharger_To_HomePosition = 22
    tCharger_To_VTOL2Charger1 = 23
    tCharger_To_VTOL2Charger2 = 24
    tCharger_To_Helicopter2Charger1 = 25
    tHelicopter2Charger1_To_Charger = 26
    tHelicopter2Charger1_To_Helicopter2Charger1Slot1 = 27
    tHelicopter2Charger1_To_Helicopter2Charger1Slot2 = 28
    tHelicopter2Charger1_To_Helicopter2Charger1BeforeSlot1 = 29
    tHelicopter2Charger1Slot1_To_Helicopter2Charger1 = 30
    tHelicopter2Charger1Slot1_To_Helicopter2Charger1InsideSlot1 = 31
    tHelicopter2Charger1Slot1_To_Helicopter2Charger1BeforeSlot1 = 32
    tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1Slot1 = 33
    tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1BeforeSlot1 = 34
    tHelicopter2Charger1BeforeSlot1_To_Helicopter2Charger1 = 35
    tHelicopter2Charger1Slot2_To_Helicopter2Charger1 = 36
    tHelicopter2Charger1Slot2_To_Helicopter2Charger1InsideSlot2 = 37
    tHelicopter2Charger1Slot2_To_Helicopter2Charger1BeforeSlot2 = 38
    tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1Slot2 = 39
    tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1BeforeSlot2 = 40
    tHelicopter2Charger1BeforeSlot2_To_Helicopter2Charger1 = 41
    tHelicopter2_To_Helicopter2Battery1 = 42
    tHelicopter2_To_Helicopter2Battery2 = 43
    tHelicopter2Battery1_To_Helicopter2 = 44
    tHelicopter2Battery1_To_Helicopter2InsideBattery1Slot = 45
    tHelicopter2Battery1_To_Helicopter2BeforeBattery1Slot = 46
    tHelicopter2InsideBattery1Slot_To_Helicopter2Battery1 = 47
    tHelicopter2InsideBattery1Slot_To_Helicopter2BeforeBattery1Slot = 48
    tHelicopter2Battery2_To_Helicopter2 = 49
    tHelicopter2Battery2_To_Helicopter2InsideBattery2Slot = 50
    tHelicopter2Battery2_To_Helicopter2BeforeBattery2Slot = 51
    tHelicopter2InsideBattery2Slot_To_Helicopter2Battery2 = 52
    tHelicopter2InsideBattery2Slot_To_Helicopter2BeforeBattery2Slot = 53
    tHomePosition_To_VTOLModule = 54
    tVTOLModule_To_HomePosition = 55
    tVTOLModule_To_VTOL1 = 56
    tVTOL1_To_VTOLModule = 57
    tVTOL1_To_VTOL1Payload = 58
    tVTOL1Payload_To_VTOL1 = 59
    tVTOLModule_To_VTOL2 = 60
    tVTOL2_To_VTOLModule = 61
    tVTOL2_To_VTOL2Battery1 = 62
    tVTOL2_To_VTOL2Battery2 = 63
    tVTOL2Battery1_To_VTOL2 = 64
    tVTOL2Battery2_To_VTOL2 = 65
    tGrippers1_To_Grippers1InsideSlot = 66
    tGrippers1InsideSlot_To_Grippers1BeforeSlot = 67
    tGrippers2_To_Grippers2InsideSlot = 68
    tGrippers2InsideSlot_To_Grippers2BeforeSlot = 69
    tVTOL1Payload_To_VTOL1InsidePayloadSlot = 70
    tVTOL1InsidePayloadSlot_To_VTOL1BeforePayloadSlot = 71
    tVTOL2Battery1_To_VTOL2InsideBattery1Slot = 72
    tVTOL2InsideBattery1Slot_To_VTOL2BeforeBattery1Slot = 73
    tVTOL2Battery2_To_VTOL2InsideBattery2Slot = 74
    tVTOL2InsideBattery2Slot_To_VTOL2BeforeBattery2Slot = 75
    tVTOL2Charger1_To_Charger = 76
    tVTOL2Charger1_To_VTOL2Charger1Slot1 = 77
    tVTOL2Charger1_To_VTOL2Charger1Slot2 = 78
    tVTOL2Charger1_To_VTOL2Charger1BeforeSlot1 = 79
    tVTOL2Charger1Slot1_To_VTOL2Charger1 = 80
    tVTOL2Charger1Slot1_To_VTOL2Charger1InsideSlot1 = 81
    tVTOL2Charger1Slot1_To_VTOL2Charger1BeforeSlot1 = 82
    tVTOL2Charger1InsideSlot1_To_VTOL2Charger1Slot1 = 83
    tVTOL2Charger1InsideSlot1_To_VTOL2Charger1BeforeSlot1 = 84
    tVTOL2Charger1Slot2_To_VTOL2Charger1 = 85
    tVTOL2Charger1Slot2_To_VTOL2Charger1InsideSlot2 = 86
    tVTOL2Charger1Slot2_To_VTOL2Charger1BeforeSlot2 = 87
    tVTOL2Charger1InsideSlot2_To_VTOL2Charger1Slot2 = 88
    tVTOL2Charger1InsideSlot2_To_VTOL2Charger1BeforeSlot2 = 89
    tVTOL2Charger2_To_Charger = 90
    tVTOL2Charger2_To_VTOL2Charger2Slot1 = 91
    tVTOL2Charger2_To_VTOL2Charger2Slot2 = 92
    tVTOL2Charger2_To_VTOL2Charger2BeforeSlot1 = 93
    tVTOL2Charger2Slot1_To_VTOL2Charger2 = 94
    tVTOL2Charger2Slot1_To_VTOL2Charger2InsideSlot1 = 95
    tVTOL2Charger2Slot1_To_VTOL2Charger2BeforeSlot1 = 96
    tVTOL2Charger2InsideSlot1_To_VTOL2Charger2Slot1 = 97
    tVTOL2Charger2InsideSlot1_To_VTOL2Charger2BeforeSlot1 = 98
    tVTOL2Charger2Slot2_To_VTOL2Charger2 = 99
    tVTOL2Charger2Slot2_To_VTOL2Charger2InsideSlot2 = 100
    tVTOL2Charger2Slot2_To_VTOL2Charger2BeforeSlot2 = 101
    tVTOL2Charger2InsideSlot2_To_VTOL2Charger2Slot2 = 102
    tVTOL2Charger2InsideSlot2_To_VTOL2Charger2BeforeSlot2 = 103
    tPayload1_To_Payload1InsideSlot = 104
    tPayload1_To_Payload1BeforeSlot = 105
    tPayload2_To_Payload2InsideSlot = 106
    tPayload2_To_Payload2BeforeSlot = 107
    tPayload1InsideSlot_To_Payload1 = 108
    tPayload2InsideSlot_To_Payload2 = 109
    tGrippers1InsideSlot_To_Grippers1 = 110
    tGrippers2InsideSlot_To_Grippers2 = 111
    tVTOL1InsideBatterySlot_To_VTOL1Battery = 112
    tVTOL1InsidePayloadSlot_To_VTOL1Payload = 113
    tVTOL2InsideBattery1Slot_To_VTOL2Battery1 = 114
    tVTOL2InsideBattery2Slot_To_VTOL2Battery2 = 115
    tHelicopter2BeforeBattery1Slot_To_Helicopter2 = 116
    tHelicopter2BeforeBattery2Slot_To_Helicopter2 = 117
    tVTOL2Charger1BeforeSlot1_To_VTOL2Charger1 = 118
    tVTOL2Charger1BeforeSlot2_To_VTOL2Charger1 = 119
    tVTOL2Charger2BeforeSlot1_To_VTOL2Charger2 = 120
    tVTOL2Charger2BeforeSlot2_To_VTOL2Charger2 = 121
    tVTOL2BeforeBattery1Slot_To_VTOL2 = 122
    tVTOL2BeforeBattery2Slot_To_VTOL2 = 123

    tOpenCap = 124

    tVTOL2Charger2Slot1_after_ejecting = 125
    tVTOL2Charger2Slot1_before_taking = 126
    tVTOL2Charger2Slot1_grab_batt = 127
    tVTOL2Charger2Slot1_take_batt = 128
    tVTOL2Charger2Slot1_remove_batt = 129
    tVTOL2Charger2Slot1_stick_in_batt = 130
    tVTOL2Charger2Slot1_put_batt = 131
    tVTOL2Charger2Slot1_grab_in_batt = 132
    tVTOL2Charger2Slot1_before_inject = 133
    tVTOL2Charger2Slot1_move_batt = 134
    tVTOL2Charger2Slot1_move_end_point_batt = 135
    tVTOL2Charger2Slot1_after_sticking_batt = 136

    tVTOL2Charger2Slot2_after_ejecting = 137
    tVTOL2Charger2Slot2_before_taking = 138
    tVTOL2Charger2Slot2_grab_batt = 139
    tVTOL2Charger2Slot2_take_batt = 140
    tVTOL2Charger2Slot2_remove_batt = 141
    tVTOL2Charger2Slot2_stick_in_batt = 142
    tVTOL2Charger2Slot2_put_batt = 143
    tVTOL2Charger2Slot2_grab_in_batt = 144
    tVTOL2Charger2Slot2_before_inject = 145
    tVTOL2Charger2Slot2_move_batt = 146
    tVTOL2Charger2Slot2_move_end_point_batt = 147
    tVTOL2Charger2Slot2_after_sticking_batt = 148


class RobotRoutes(enum.Enum):
    rHomePosition_To_Helicopter1 = 1


class RobotActions(enum.Enum):
    aVTOL2_To_VTOL2Battery = 1
    aVTOL2Battery_To_VTOL2 = 2
    aCharger_To_Helicopter2Batt1 = 3
