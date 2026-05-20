# available_trajectories.py

available_trajectories = {
    "pHomePosition": [
        "tHomePosition_To_HelicopterModule",
        "tHomePosition_To_Payload",
        "tHomePosition_To_Grippers",
        "tHomePosition_To_Charger",
        "tHomePosition_To_VTOLModule",
    ],
    "pHelicopterModule": [
        "tHelicopterModule_To_HomePosition",
        "tHelicopterModule_To_Helicopter1",
        "tHelicopterModule_To_Helicopter2",
        "tHelicopterModule_To_HelicopterModuleWithSomeCargo",
    ],
    "pHelicopter1": [
        "tHelicopter1_To_HelicopterModule",
    ],
    "pHelicopter2": [
        "tHelicopter2_To_HelicopterModule",
        "tHelicopter2_To_Helicopter2Payload",
        "tHelicopter2_To_Helicopter2Battery1",
        "tHelicopter2_To_Helicopter2Battery2",
    ],
    "pHelicopter2Payload": [
        "tHelicopter2Payload_To_Helicopter2",
    ],
    "pHelicopter2Battery1": [
        "tHelicopter2Battery1_To_Helicopter2",
        "tHelicopter2Battery1_To_Helicopter2InsideBattery1Slot",
        "tHelicopter2Battery1_To_Helicopter2BeforeBattery1Slot",
    ],
    "pHelicopter2InsideBattery1Slot": [
        "tHelicopter2InsideBattery1Slot_To_Helicopter2Battery1",
        "tHelicopter2InsideBattery1Slot_To_Helicopter2BeforeBattery1Slot",
    ],
    "pHelicopter2BeforeBattery1Slot": [
        "tHelicopter2BeforeBattery1Slot_To_Helicopter2",
        "tHelicopter2BeforeBattery1Slot_To_Helicopter2WithSomeCargo",
    ],
    "pHelicopter2Battery2": [
        "tHelicopter2Battery2_To_Helicopter2",
        "tHelicopter2Battery2_To_Helicopter2InsideBattery2Slot",
        "tHelicopter2Battery2_To_Helicopter2BeforeBattery2Slot",
    ],
    "pHelicopter2InsideBattery2Slot": [
        "tHelicopter2InsideBattery2Slot_To_Helicopter2Battery2",
        "tHelicopter2InsideBattery2Slot_To_Helicopter2BeforeBattery2Slot",
    ],
    "pHelicopter2BeforeBattery2Slot": [
        "tHelicopter2BeforeBattery2Slot_To_Helicopter2",
        "tHelicopter2BeforeBattery2Slot_To_Helicopter2WithSomeCargo",
    ],
    "pHelicopter2WithSomeCargo": [
        "tHelicopter2WithSomeCargo_To_Helicopter2WithBatt1",
        "tHelicopter2WithSomeCargo_To_Helicopter2WithBatt2",
        "tHelicopter2WithSomeCargo_To_Helicopter2AfterSlot1",
        "tHelicopter2WithSomeCargo_To_Helicopter2AfterSlot2",
    ],
    "pHelicopter2WithBatt1": [
        "tHelicopter2WithBatt1_To_Helicopter2WithSomeCargo",
        "tHelicopter2WithBatt1_To_Helicopter2InSlot1",
        "tHelicopter2WithBatt1_To_Helicopter2AfterSlot1",
    ],
    "pHelicopter2InSlot1": [
        "tHelicopter2InSlot1_To_Helicopter2WithBatt1",
        "tHelicopter2InSlot1_To_Helicopter2AfterSlot1",
    ],
    "pHelicopter2AfterSlot1": [
        "tHelicopter2AfterSlot1_To_Helicopter2WithSomeCargo",
    ],
    "pHelicopter2WithBatt2": [
        "tHelicopter2WithBatt2_To_Helicopter2WithSomeCargo",
        "tHelicopter2WithBatt2_To_Helicopter2InSlot2",
        "tHelicopter2WithBatt2_To_Helicopter2AfterSlot2",
    ],
    "pHelicopter2InSlot2": [
        "tHelicopter2InSlot2_To_Helicopter2WithBatt2",
        "tHelicopter2InSlot2_To_Helicopter2AfterSlot2",
    ],
    "pHelicopter2AfterSlot2": [
        "tHelicopter2AfterSlot2_To_Helicopter2WithSomeCargo",
    ],
    "pPayload": [
        "tPayload_To_HomePosition",
        "tPayload_To_Payload1",
        "tPayload_To_Payload2",
    ],
    "pPayload1": [
        "tPayload1_To_Payload",
        "tPayload1_To_Payload1InsideSlot",
        "tPayload1_To_Payload1BeforeSlot",
    ],
    "pPayload1InsideSlot": [
        "tPayload1InsideSlot_To_Payload1BeforeSlot",
    ],
    "pPayload1BeforeSlot": [
        "tPayload1BeforeSlot_To_Payload1",
    ],
    "pPayload2": [
        "tPayload2_To_Payload",
        "tPayload2_To_Payload2InsideSlot",
        "tPayload2_To_Payload2BeforeSlot",
    ],
    "pPayload2InsideSlot": [
        "tPayload2InsideSlot_To_Payload2BeforeSlot",
    ],
    "pPayload2BeforeSlot": [
        "tPayload2BeforeSlot_To_Payload2",
    ],
    "pGrippers": [
        "tGrippers_To_HomePosition",
        "tGrippers_To_Grippers1",
        "tGrippers_To_Grippers2",
    ],
    "pGrippers1": [
        "tGrippers1_To_Grippers",
        "tGrippers1_To_Grippers1InsideSlot",
    ],
    "pGrippers1InsideSlot": [
        "tGrippers1InsideSlot_To_Grippers1BeforeSlot",
    ],
    "pGrippers1BeforeSlot": [
        "tGrippers1BeforeSlot_To_Grippers1",
    ],
    "pGrippers2": [
        "tGrippers2_To_Grippers",
        "tGrippers2_To_Grippers2InsideSlot",
    ],
    "pGrippers2InsideSlot": [
        "tGrippers2InsideSlot_To_Grippers2BeforeSlot",
    ],
    "pGrippers2BeforeSlot": [
        "tGrippers2BeforeSlot_To_Grippers2",
    ],
    "pCharger": [
        "tCharger_To_HomePosition",
        "tCharger_To_VTOL2Charger1",
        "tCharger_To_VTOL2Charger2",
        "tCharger_To_Helicopter2Charger1",
    ],
    "pHelicopter2Charger1": [
        "tHelicopter2Charger1_To_Charger",
        "tHelicopter2Charger1_To_Helicopter2Charger1Slot1",
        "tHelicopter2Charger1_To_Helicopter2Charger1Slot2",
        "tHelicopter2Charger1_To_Helicopter2Charger1BeforeSlot1",
    ],
    "pHelicopter2Charger1Slot1": [
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1",
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1InsideSlot1",
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1BeforeSlot1",
    ],
    "pHelicopter2Charger1InsideSlot1": [
        "tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1Slot1",
        "tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1BeforeSlot1",
    ],
    "pHelicopter2Charger1BeforeSlot1": [
        "tHelicopter2Charger1BeforeSlot1_To_Helicopter2Charger1",
    ],
    "pHelicopter2Charger1Slot2": [
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1",
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1InsideSlot2",
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1BeforeSlot2",
    ],
    "pHelicopter2Charger1InsideSlot2": [
        "tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1Slot2",
        "tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1BeforeSlot2",
    ],
    "pHelicopter2Charger1BeforeSlot2": [
        "tHelicopter2Charger1BeforeSlot2_To_Helicopter2Charger1",
    ],
    "pVTOL2Charger1": [
        "tVTOL2Charger1_To_Charger",
        "tVTOL2Charger1_To_VTOL2Charger1Slot1",
        "tVTOL2Charger1_To_VTOL2Charger1Slot2",
        "tVTOL2Charger1_To_VTOL2Charger1BeforeSlot1",
    ],
    "pVTOL2Charger1Slot1": [
        "tVTOL2Charger1Slot1_To_VTOL2Charger1",
        "tVTOL2Charger1Slot1_To_VTOL2Charger1InsideSlot1",
        "tVTOL2Charger1Slot1_To_VTOL2Charger1BeforeSlot1",
    ],
    "pVTOL2Charger1InsideSlot1": [
        "tVTOL2Charger1InsideSlot1_To_VTOL2Charger1Slot1",
        "tVTOL2Charger1InsideSlot1_To_VTOL2Charger1BeforeSlot1",
    ],
    "pVTOL2Charger1BeforeSlot1": [
        "tVTOL2Charger1BeforeSlot1_To_VTOL2Charger1",
    ],
    "pVTOL2Charger1Slot2": [
        "tVTOL2Charger1Slot2_To_VTOL2Charger1",
        "tVTOL2Charger1Slot2_To_VTOL2Charger1InsideSlot2",
        "tVTOL2Charger1Slot2_To_VTOL2Charger1BeforeSlot2",
    ],
    "pVTOL2Charger1InsideSlot2": [
        "tVTOL2Charger1InsideSlot2_To_VTOL2Charger1Slot2",
        "tVTOL2Charger1InsideSlot2_To_VTOL2Charger1BeforeSlot2",
    ],
    "pVTOL2Charger1BeforeSlot2": [
        "tVTOL2Charger1BeforeSlot2_To_VTOL2Charger1",
    ],
    "pVTOL2Charger2": [
        "tVTOL2Charger2_To_Charger",
        "tVTOL2Charger2_To_VTOL2Charger2Slot1",
        "tVTOL2Charger2_To_VTOL2Charger2Slot2",
        "tVTOL2Charger2_To_VTOL2Charger2BeforeSlot1",
    ],
    "pVTOL2Charger2Slot1": [
        "tVTOL2Charger2Slot1_To_VTOL2Charger2",
        "tVTOL2Charger2Slot1_To_VTOL2Charger2InsideSlot1",
        "tVTOL2Charger2Slot1_To_VTOL2Charger2BeforeSlot1",
    ],
    "pVTOL2Charger2InsideSlot1": [
        "tVTOL2Charger2InsideSlot1_To_VTOL2Charger2Slot1",
        "tVTOL2Charger2InsideSlot1_To_VTOL2Charger2BeforeSlot1",
    ],
    "pVTOL2Charger2BeforeSlot1": [
        "tVTOL2Charger2BeforeSlot1_To_VTOL2Charger2",
    ],
    "pVTOL2Charger2Slot2": [
        "tVTOL2Charger2Slot2_To_VTOL2Charger2",
        "tVTOL2Charger2Slot2_To_VTOL2Charger2InsideSlot2",
        "tVTOL2Charger2Slot2_To_VTOL2Charger2BeforeSlot2",
    ],
    "pVTOL2Charger2InsideSlot2": [
        "tVTOL2Charger2InsideSlot2_To_VTOL2Charger2Slot2",
        "tVTOL2Charger2InsideSlot2_To_VTOL2Charger2BeforeSlot2",
    ],
    "pVTOL2Charger2BeforeSlot2": [
        "tVTOL2Charger2BeforeSlot2_To_VTOL2Charger2",
    ],
    "pVTOLModule": [
        "tVTOLModule_To_HomePosition",
        "tVTOLModule_To_VTOL1",
        "tVTOLModule_To_VTOL2",
    ],
    "pVTOL1": [
        "tVTOL1_To_VTOLModule",
        "tVTOL1_To_VTOL1Payload",
    ],
    "pVTOL1Payload": [
        "tVTOL1Payload_To_VTOL1",
        "tVTOL1Payload_To_VTOL1InsidePayloadSlot",
    ],
    "pVTOL1InsidePayloadSlot": [
        "tVTOL1InsidePayloadSlot_To_VTOL1BeforePayloadSlot",
    ],
    "pVTOL1BeforePayloadSlot": [
        "tVTOL1BeforePayloadSlot_To_VTOL1Payload",
    ],
    "pVTOL2": [
        "tVTOL2_To_VTOLModule",
        "tVTOL2_To_VTOL2Battery1",
        "tVTOL2_To_VTOL2Battery2",
    ],
    "pVTOL2Battery1": [
        "tVTOL2Battery1_To_VTOL2",
        "tVTOL2Battery1_To_VTOL2InsideBattery1Slot",
    ],
    "pVTOL2InsideBattery1Slot": [
        "tVTOL2InsideBattery1Slot_To_VTOL2BeforeBattery1Slot",
    ],
    "pVTOL2BeforeBattery1Slot": [
        "tVTOL2BeforeBattery1Slot_To_VTOL2",
    ],
    "pVTOL2Battery2": [
        "tVTOL2Battery2_To_VTOL2",
        "tVTOL2Battery2_To_VTOL2InsideBattery2Slot",
    ],
    "pVTOL2InsideBattery2Slot": [
        "tVTOL2InsideBattery2Slot_To_VTOL2BeforeBattery2Slot",
    ],
    "pVTOL2BeforeBattery2Slot": [
        "tVTOL2BeforeBattery2Slot_To_VTOL2",
    ],
    "pVTOL2Battery2Charge": [
        "tVTOL2Battery2_To_VTOL2Battery2Charge",
    ],
    "pHelicopterModuleWithSomeCargo": [
        "tHelicopterModuleWithSomeCargo_To_HelicopterModule",
    ],
}