available_trajectories = {
    # ===== БАЗОВЫЕ ТОЧКИ (без груза) =====
    "pHomePosition": {
        "tHomePosition_To_HelicopterModule",
        "tHomePosition_To_Payload",
        "tHomePosition_To_Grippers",
        "tHomePosition_To_Charger",
        "tHomePosition_To_VTOLModule",
        "tHomePosition_To_HelicopterModuleWithSomeCargo",
        "tHomePosition_To_PayloadWithPL",
        "tHomePosition_To_VTOLModuleWithSomeCargo",
        "tHomePosition_To_ChargerWithBatt",
    },
    "pHelicopterModule": {
        "tHelicopterModule_To_HomePosition",
        "tHelicopterModule_To_Helicopter1",
        "tHelicopterModule_To_Helicopter2",
        "tHelicopterModule_To_HomePositionWithSomeCargo",
        "tHelicopterModule_To_Helicopter1WithPL",
        "tHelicopterModule_To_Helicopter2WithPL",
        "tHelicopterModule_To_HelicopterModuleWithSomeCargo",
    },
    "pHelicopter1": {
        "tHelicopter1_To_HelicopterModule",
        "tHelicopter1_To_Helicopter1Payload",
    },
    "pHelicopter1Payload": {
        "tHelicopter1Payload_To_Helicopter1",
        "tHelicopter1Payload_To_Helicopter1InsideSlot",
        "tHelicopter1Payload_To_Helicopter1BeforeSlot",
    },
    "pHelicopter1InsideSlot": {
        "tHelicopter1InsideSlot_To_Helicopter1BeforeSlot",
    },
    "pHelicopter1BeforeSlot": {
        "tHelicopter1BeforeSlot_To_Helicopter1WithPL",
    },
    "pHelicopter2": {
        "tHelicopter2_To_HelicopterModule",
        "tHelicopter2_To_Helicopter2Payload",
        "tHelicopter2_To_Helicopter2Battery1",
        "tHelicopter2_To_Helicopter2Battery2",
    },
    "pHelicopter2Payload": {
        "tHelicopter2Payload_To_Helicopter2",
    },
    "pHelicopter2Battery1": {
        "tHelicopter2Battery1_To_Helicopter2",
        "tHelicopter2Battery1_To_Helicopter2InsideBattery1Slot",
        "tHelicopter2Battery1_To_Helicopter2BeforeBattery1Slot",
    },
    "pHelicopter2InsideBattery1Slot": {
        "tHelicopter2InsideBattery1Slot_To_Helicopter2Battery1",
        "tHelicopter2InsideBattery1Slot_To_Helicopter2BeforeBattery1Slot",
    },
    "pHelicopter2BeforeBattery1Slot": {
        "tHelicopter2BeforeBattery1Slot_To_Helicopter2WithSomeCargo",
    },
    "pHelicopter2Battery2": {
        "tHelicopter2Battery2_To_Helicopter2",
        "tHelicopter2Battery2_To_Helicopter2InsideBattery2Slot",
        "tHelicopter2Battery2_To_Helicopter2BeforeBattery2Slot",
    },
    "pHelicopter2InsideBattery2Slot": {
        "tHelicopter2InsideBattery2Slot_To_Helicopter2Battery2",
        "tHelicopter2InsideBattery2Slot_To_Helicopter2BeforeBattery2Slot",
    },
    "pHelicopter2BeforeBattery2Slot": {
        "tHelicopter2BeforeBattery2Slot_To_Helicopter2WithSomeCargo",
    },
    "pHelicopter2WithSomeCargo": {
        "tHelicopter2WithSomeCargo_To_Helicopter2WithBatt1",
        "tHelicopter2WithSomeCargo_To_Helicopter2WithBatt2",
        "tHelicopter2WithSomeCargo_To_Helicopter2PayloadWithPL",
        "tHelicopter2WithSomeCargo_To_Helicopter2AfterSlot1",
        "tHelicopter2WithSomeCargo_To_Helicopter2AfterSlot2",
    },
    "pHelicopter2WithBatt1": {
        "tHelicopter2WithBatt1_To_Helicopter2WithSomeCargo",
        "tHelicopter2WithBatt1_To_Helicopter2InSlot1",
        "tHelicopter2WithBatt1_To_Helicopter2AfterSlot1",
    },
    "pHelicopter2InSlot1": {
        "tHelicopter2InSlot1_To_Helicopter2WithBatt1",
        "tHelicopter2InSlot1_To_Helicopter2AfterSlot1",
    },
    "pHelicopter2AfterSlot1": {
        "tHelicopter2AfterSlot1_To_Helicopter2WithSomeCargo",
    },
    "pHelicopter2WithBatt2": {
        "tHelicopter2WithBatt2_To_Helicopter2WithSomeCargo",
        "tHelicopter2WithBatt2_To_Helicopter2InSlot2",
        "tHelicopter2WithBatt2_To_Helicopter2AfterSlot2",
    },
    "pHelicopter2InSlot2": {
        "tHelicopter2InSlot2_To_Helicopter2WithBatt2",
        "tHelicopter2InSlot2_To_Helicopter2AfterSlot2",
    },
    "pHelicopter2AfterSlot2": {
        "tHelicopter2AfterSlot2_To_Helicopter2WithSomeCargo",
    },
    "pPayload": {
        "tPayload_To_HomePosition",
        "tPayload_To_Payload1",
        "tPayload_To_Payload2",
        "tPayload_To_HomePositionWithSomeCargo",
        "tPayload_To_PayloadWithPL",
    },
    "pPayload1": {
        "tPayload1_To_Payload",
        "tPayload1_To_Payload1InsideSlot",
        "tPayload1_To_Payload1BeforeSlot",
    },
    "pPayload1InsideSlot": {
        "tPayload1InsideSlot_To_Payload1BeforeSlot",
    },
    "pPayload1BeforeSlot": {
        "tPayload1BeforeSlot_To_Payload1",
    },
    "pPayload2": {
        "tPayload2_To_Payload",
        "tPayload2_To_Payload2InsideSlot",
        "tPayload2_To_Payload2BeforeSlot",
    },
    "pPayload2InsideSlot": {
        "tPayload2InsideSlot_To_Payload2BeforeSlot",
    },
    "pPayload2BeforeSlot": {
        "tPayload2BeforeSlot_To_Payload2",
    },
    "pGrippers": {
        "tGrippers_To_HomePosition",
        "tGrippers_To_Grippers1",
        "tGrippers_To_Grippers2",
        "tGrippers_To_GrippersWithGrip",
    },
    "pGrippers1": {
        "tGrippers1_To_Grippers",
        "tGrippers1_To_Grippers1InsideSlot",
    },
    "pGrippers1InsideSlot": {
        "tGrippers1InsideSlot_To_Grippers1BeforeSlot",
    },
    "pGrippers1BeforeSlot": {
        "tGrippers1BeforeSlot_To_Grippers1",
    },
    "pGrippers2": {
        "tGrippers2_To_Grippers",
        "tGrippers2_To_Grippers2InsideSlot",
    },
    "pGrippers2InsideSlot": {
        "tGrippers2InsideSlot_To_Grippers2BeforeSlot",
    },
    "pGrippers2BeforeSlot": {
        "tGrippers2BeforeSlot_To_Grippers2",
    },
    "pCharger": {
        "tCharger_To_HomePosition",
        "tCharger_To_VTOL2Charger1",
        "tCharger_To_VTOL2Charger2",
        "tCharger_To_Helicopter2Charger1",
        "tCharger_To_ChargerWithBatt",
    },
    "pHelicopter2Charger1": {
        "tHelicopter2Charger1_To_Charger",
        "tHelicopter2Charger1_To_Helicopter2Charger1Slot1",
        "tHelicopter2Charger1_To_Helicopter2Charger1Slot2",
        "tHelicopter2Charger1_To_Helicopter2Charger1BeforeSlot1",
    },
    "pHelicopter2Charger1Slot1": {
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1",
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1InsideSlot1",
        "tHelicopter2Charger1Slot1_To_Helicopter2Charger1BeforeSlot1",
    },
    "pHelicopter2Charger1InsideSlot1": {
        "tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1Slot1",
        "tHelicopter2Charger1InsideSlot1_To_Helicopter2Charger1BeforeSlot1",
    },
    "pHelicopter2Charger1BeforeSlot1": {
        "tHelicopter2Charger1BeforeSlot1_To_Helicopter2Charger1",
    },
    "pHelicopter2Charger1Slot2": {
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1",
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1InsideSlot2",
        "tHelicopter2Charger1Slot2_To_Helicopter2Charger1BeforeSlot2",
    },
    "pHelicopter2Charger1InsideSlot2": {
        "tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1Slot2",
        "tHelicopter2Charger1InsideSlot2_To_Helicopter2Charger1BeforeSlot2",
    },
    "pHelicopter2Charger1BeforeSlot2": {
        "tHelicopter2Charger1BeforeSlot2_To_Helicopter2Charger1",
    },
    "pVTOL2Charger1": {
        "tVTOL2Charger1_To_Charger",
        "tVTOL2Charger1_To_VTOL2Charger1Slot1",
        "tVTOL2Charger1_To_VTOL2Charger1Slot2",
        "tVTOL2Charger1_To_VTOL2Charger1BeforeSlot1",
    },
    "pVTOL2Charger1Slot1": {
        "tVTOL2Charger1Slot1_To_VTOL2Charger1",
        "tVTOL2Charger1Slot1_To_VTOL2Charger1InsideSlot1",
        "tVTOL2Charger1Slot1_To_VTOL2Charger1BeforeSlot1",
    },
    "pVTOL2Charger1InsideSlot1": {
        "tVTOL2Charger1InsideSlot1_To_VTOL2Charger1Slot1",
        "tVTOL2Charger1InsideSlot1_To_VTOL2Charger1BeforeSlot1",
    },
    "pVTOL2Charger1BeforeSlot1": {
        "tVTOL2Charger1BeforeSlot1_To_VTOL2Charger1WithBatt",
    },
    "pVTOL2Charger1Slot2": {
        "tVTOL2Charger1Slot2_To_VTOL2Charger1",
        "tVTOL2Charger1Slot2_To_VTOL2Charger1InsideSlot2",
        "tVTOL2Charger1Slot2_To_VTOL2Charger1BeforeSlot2",
    },
    "pVTOL2Charger1InsideSlot2": {
        "tVTOL2Charger1InsideSlot2_To_VTOL2Charger1Slot2",
        "tVTOL2Charger1InsideSlot2_To_VTOL2Charger1BeforeSlot2",
    },
    "pVTOL2Charger1BeforeSlot2": {
        "tVTOL2Charger1BeforeSlot2_To_VTOL2Charger1WithBatt",
    },
    "pVTOL2Charger2": {
        "tVTOL2Charger2_To_Charger",
        "tVTOL2Charger2_To_VTOL2Charger2Slot1",
        "tVTOL2Charger2_To_VTOL2Charger2Slot2",
        "tVTOL2Charger2_To_VTOL2Charger2BeforeSlot1",
    },
    "pVTOL2Charger2Slot1": {
        "tVTOL2Charger2Slot1_To_VTOL2Charger2",
        "tVTOL2Charger2Slot1_To_VTOL2Charger2InsideSlot1",
        "tVTOL2Charger2Slot1_To_VTOL2Charger2BeforeSlot1",
    },
    "pVTOL2Charger2InsideSlot1": {
        "tVTOL2Charger2InsideSlot1_To_VTOL2Charger2Slot1",
        "tVTOL2Charger2InsideSlot1_To_VTOL2Charger2BeforeSlot1",
    },
    "pVTOL2Charger2BeforeSlot1": {
        "tVTOL2Charger2BeforeSlot1_To_VTOL2Charger2WithBatt",
    },
    "pVTOL2Charger2Slot2": {
        "tVTOL2Charger2Slot2_To_VTOL2Charger2",
        "tVTOL2Charger2Slot2_To_VTOL2Charger2InsideSlot2",
        "tVTOL2Charger2Slot2_To_VTOL2Charger2BeforeSlot2",
    },
    "pVTOL2Charger2InsideSlot2": {
        "tVTOL2Charger2InsideSlot2_To_VTOL2Charger2Slot2",
        "tVTOL2Charger2InsideSlot2_To_VTOL2Charger2BeforeSlot2",
    },
    "pVTOL2Charger2BeforeSlot2": {
        "tVTOL2Charger2BeforeSlot2_To_VTOL2Charger2WithBatt",
    },
    "pVTOLModule": {
        "tVTOLModule_To_HomePosition",
        "tVTOLModule_To_VTOL1",
        "tVTOLModule_To_VTOL2",
        "tVTOLModule_To_HomePositionWithSomeCargo",
        "tVTOLModule_To_VTOLModuleWithSomeCargo",
    },
    "pVTOL1": {
        "tVTOL1_To_VTOLModule",
        "tVTOL1_To_VTOL1Payload",
        "tVTOL1_To_VTOL1Battery",
    },
    "pVTOL1Payload": {
        "tVTOL1Payload_To_VTOL1",
        "tVTOL1Payload_To_VTOL1InsidePayloadSlot",
    },
    "pVTOL1InsidePayloadSlot": {
        "tVTOL1InsidePayloadSlot_To_VTOL1BeforePayloadSlot",
    },
    "pVTOL1BeforePayloadSlot": {
        "tVTOL1BeforePayloadSlot_To_VTOL1Payload",
    },
    "pVTOL1Battery": {
        "tVTOL1Battery_To_VTOL1",
        "tVTOL1Battery_To_VTOL1InsideBatterySlot",
    },
    "pVTOL1InsideBatterySlot": {
        "tVTOL1InsideBattery1Slot_To_VTOL1BeforeBatterySlot",
    },
    "pVTOL1BeforeBatterySlot": {
        "tVTOL1BeforeBatterySlot_To_VTOL1",
    },
    "pVTOL2": {
        "tVTOL2_To_VTOLModule",
        "tVTOL2_To_VTOL2Battery1",
        "tVTOL2_To_VTOL2Battery2",
        "tVTOL2Battery_To_VTOL2",
    },
    "pVTOL2Battery1": {
        "tVTOL2Battery1_To_VTOL2",
        "tVTOL2Battery1_To_VTOL2InsideBattery1Slot",
    },
    "pVTOL2InsideBattery1Slot": {
        "tVTOL2InsideBattery1Slot_To_VTOL2BeforeBattery1Slot",
    },
    "pVTOL2BeforeBattery1Slot": {
        "tVTOL2BeforeBattery1Slot_To_VTOL2Battery1",
    },
    "pVTOL2Battery2": {
        "tVTOL2Battery2_To_VTOL2",
        "tVTOL2Battery2_To_VTOL2InsideBattery2Slot",
        "tVTOL2Battery2_To_VTOL2_withVTOL2Batt2",
    },
    "pVTOL2InsideBattery2Slot": {
        "tVTOL2InsideBattery2Slot_To_VTOL2BeforeBattery2Slot",
    },
    "pVTOL2BeforeBattery2Slot": {
        "tVTOL2BeforeBattery2Slot_To_VTOL2Battery2",
    },
    "pVTOL2Battery2Charge": {
        "tVTOL2Battery2Charge_To_VTOL2Battery2",  # резерв
        "tVTOL2Battery2_To_VTOL2Battery2Charge",
    },

    # ===== ТОЧКИ С ГРУЗОМ =====
    "pHomePositionWithSomeCargo": {
        "tHomePosition_To_HelicopterModuleWithSomeCargo",
        "tHomePosition_To_PayloadWithPL",
        "tHomePosition_To_VTOLModuleWithSomeCargo",
        "tHomePosition_To_ChargerWithBatt",
    },
    "pHelicopterModuleWithSomeCargo": {
        "tHelicopterModule_To_HomePositionWithSomeCargo",
        "tHelicopterModule_To_Helicopter1WithPL",
        "tHelicopterModule_To_Helicopter2WithPL",
        "tHelicopterModuleWithSomeCargo_To_HelicopterModule",
    },
    "pHelicopter1WithPL": {
        "tHelicopter1WithPL_To_HelicopterModuleWithSomeCargo",
        "tHelicopter1WithPL_To_Helicopter1PayloadWithPL",
    },
    "pHelicopter1PayloadWithPL": {
        "tHelicopter1PayloadWithPL_To_Helicopter1InSlot",
    },
    "pHelicopter1InSlot": {
        "tHelicopter1InSlot_To_Helicopter1AfterSlot",
    },
    "pHelicopter1AfterSlot": {
        "tHelicopter1AfterSlot_To_Helicopter1WithPL",
    },
    "pHelicopter2PayloadWithPL": {
        "tHelicopter2PayloadWithPL_To_Helicopter2WithSomeCargo",
    },
    "pPayloadWithPL": {
        "tPayloadWithPL_To_HomePositionWithSomeCargo",
        "tPayloadWithPL_To_Payload1WithPL",
        "tPayloadWithPL_To_Payload2WithPL",
        "tPayloadWithPL_To_Payload",
    },
    "pPayload1WithPL": {
        "tPayload1WithPL_To_Payload1InSlot",
    },
    "pPayload1InSlot": {
        "tPayload1InSlot_To_Payload1AfterSlot",
    },
    "pPayload1AfterSlot": {
        "tPayload1AfterSlot_To_Payload1WithPL",
    },
    "pPayload2WithPL": {
        "tPayload2WithPL_To_Payload2InSlot",
    },
    "pPayload2InSlot": {
        "tPayload2InSlot_To_Payload2AfterSlot",
    },
    "pPayload2AfterSlot": {
        "tPayload2AfterSlot_To_Payload2WithPL",
    },
    "pGrippersWithGrip": {
        "tGrippersWithGrip_To_Grippers1WithGrip",
        "tGrippersWithGrip_To_Grippers2WithGrip",
        "tGrippersWithGrip_To_Grippers",
    },
    "pGrippers1WithGrip": {
        "tGrippers1WithGrip_To_Grippers1InSlot",
    },
    "pGrippers1InSlot": {
        "tGrippers1InSlot_To_Grippers1AfterSlot",
    },
    "pGrippers1AfterSlot": {
        "tGrippers1AfterSlot_To_Grippers1WithGrip",
    },
    "pGrippers2WithGrip": {
        "tGrippers2WithGrip_To_Grippers2InSlot",
    },
    "pGrippers2InSlot": {
        "tGrippers2InSlot_To_Grippers2AfterSlot",
    },
    "pGrippers2AfterSlot": {
        "tGrippers2AfterSlot_To_Grippers2WithGrip",
    },
    "pChargerWithBatt": {
        "tChargerWithBatt_To_Helicopter2Charger1WithBatt",
        "tChargerWithBatt_To_Charger",
    },
    "pHelicopter2Charger1WithBatt": {
        "tHelicopter2Charger1WithBatt_To_Helicopter2Charger1Slot1WithBatt",
        "tHelicopter2Charger1WithBatt_To_Helicopter2Charger1Slot2WithBatt",
        "tHelicopter2Charger1WithBatt_To_ChargerWithBatt",
    },
    "pHelicopter2Charger1Slot1WithBatt": {
        "tHelicopter2Charger1Slot1WithBatt_To_ChargerWithBatt",
        "tHelicopter2Charger1Slot1WithBatt_To_Helicopter2Charger1InSlot1",
        "tHelicopter2Charger1Slot1WithBatt_To_Helicopter2Charger1AfterSlot1",
    },
    "pHelicopter2Charger1InSlot1": {
        "tHelicopter2Charger1InSlot1_To_Helicopter2Charger1Slot1WithBatt",
        "tHelicopter2Charger1InSlot1_To_Helicopter2Charger1AfterSlot1",
    },
    "pHelicopter2Charger1AfterSlot1": {
        "tHelicopter2Charger1AfterSlot1_To_ChargerWithBatt",
    },
    "pHelicopter2Charger1Slot2WithBatt": {
        "tHelicopter2Charger1Slot2WithBatt_To_ChargerWithBatt",
        "tHelicopter2Charger1Slot2WithBatt_To_Helicopter2Charger1InSlot2",
        "tHelicopter2Charger1Slot2WithBatt_To_Helicopter2Charger1AfterSlot2",
    },
    "pHelicopter2Charger1InSlot2": {
        "tHelicopter2Charger1InSlot2_To_Helicopter2Charger1Slot2WithBatt",
        "tHelicopter2Charger1InSlot2_To_Helicopter2Charger1AfterSlot2",
    },
    "pHelicopter2Charger1AfterSlot2": {
        "tHelicopter2Charger1AfterSlot2_To_ChargerWithBatt",
    },
    "pVTOL2Charger1WithBatt": {
        "tVTOL2Charger1WithBatt_To_ChargerWithBatt",
    },
    "pVTOL2Charger1Slot1WithBatt": {
        "tVTOL2Charger1Slot1WithBatt_To_pVTOL2Charger1WithBatt",
        "tVTOL2Charger1Slot1WithBatt_To_VTOL2Charger1InSlot1",
        "tVTOL2Charger1Slot1WithBatt_To_VTOL2Charger1AfterSlot1",
    },
    "pVTOL2Charger1InSlot1": {
        "tVTOL2Charger1InSlot1_To_VTOL2Charger1Slot1WithBatt",
        "tVTOL2Charger1InSlot1_To_VTOL2Charger1AfterSlot1",
    },
    "pVTOL2Charger1AfterSlot1": {
        "tVTOL2Charger1AfterSlot1_To_VTOL2Charger1WithBatt",
    },
    "pVTOL2Charger1Slot2WithBatt": {
        "tVTOL2Charger1Slot2WithBatt_To_pVTOL2Charger1WithBatt",
        "tVTOL2Charger1Slot2WithBatt_To_VTOL2Charger1InSlot2",
        "tVTOL2Charger1Slot2WithBatt_To_VTOL2Charger1AfterSlot2",
    },
    "pVTOL2Charger1InSlot2": {
        "tVTOL2Charger1InSlot2_To_VTOL2Charger1Slot2WithBatt",
        "tVTOL2Charger1InSlot2_To_VTOL2Charger1AfterSlot2",
    },
    "pVTOL2Charger1AfterSlot2": {
        "tVTOL2Charger1AfterSlot2_To_VTOL2Charger1WithBatt",
    },
    "pVTOL2Charger2WithBatt": {
        "tVTOL2Charger2WithBatt_To_ChargerWithBatt",
    },
    "pVTOL2Charger2Slot1WithBatt": {
        "tVTOL2Charger2Slot1WithBatt_To_pVTOL2Charger2WithBatt",
        "tVTOL2Charger2Slot1WithBatt_To_VTOL2Charger2InSlot1",
        "tVTOL2Charger2Slot1WithBatt_To_VTOL2Charger2AfterSlot1",
    },
    "pVTOL2Charger2InSlot1": {
        "tVTOL2Charger2InSlot1_To_VTOL2Charger2Slot1WithBatt",
        "tVTOL2Charger2InSlot1_To_VTOL2Charger2AfterSlot1",
    },
    "pVTOL2Charger2AfterSlot1": {
        "tVTOL2Charger2AfterSlot1_To_VTOL2Charger2WithBatt",
    },
    "pVTOL2Charger2Slot2WithBatt": {
        "tVTOL2Charger2Slot2WithBatt_To_pVTOL2Charger2WithBatt",
        "tVTOL2Charger2Slot2WithBatt_To_VTOL2Charger2InSlot2",
        "tVTOL2Charger2Slot2WithBatt_To_VTOL2Charger2AfterSlot2",
    },
    "pVTOL2Charger2InSlot2": {
        "tVTOL2Charger2InSlot2_To_VTOL2Charger2Slot2WithBatt",
        "tVTOL2Charger2InSlot2_To_VTOL2Charger2AfterSlot2",
    },
    "pVTOL2Charger2AfterSlot2": {
        "tVTOL2Charger2AfterSlot2_To_VTOL2Charger2WithBatt",
    },
    "pVTOLModuleWithSomeCargo": {
        "tVTOLModule_To_HomePositionWithSomeCargo",
        "tVTOLModuleWithSomeCargo_To_VTOLModule",
    },
    "pVTOL1WithSomeCargo": {
        "tVTOL1WithSomeCargo_To_VTOL1BatteryWithBatt",
        "tVTOL1WithSomeCargo_To_VTOL1AfterBatterySlot",
    },
    "pVTOL1BatteryWithBatt": {
        "tVTOL1BatteryWithBatt_To_VTOL1InBatterySlot",
    },
    "pVTOL1InBatterySlot": {
        "tVTOL1InBattery1Slot_To_VTOL1AfterBatterySlot",
    },
    "pVTOL1AfterBatterySlot": {
        "tVTOL1AfterBatterySlot_To_VTOL1WithSomeCargo",
    },
    "pVTOL2WithBatt": {
        "tVTOL2WithBatt_To_VTOL2Battery1WithBatt",
        "tVTOL2WithBatt_To_VTOL2Battery2WithBatt",
        "tVTOL2WithBatt_To_VTOL2AfterBattery1Slot",
        "tVTOL2WithBatt_To_VTOL2AfterBattery2Slot",
    },
    "pVTOL2Battery1WithBatt": {
        "tVTOL2Battery1WithBatt_To_VTOL2InBattery1Slot",
    },
    "pVTOL2InBattery1Slot": {
        "tVTOL2InBattery1Slot_To_VTOL2AfterBattery1Slot",
    },
    "pVTOL2AfterBattery1Slot": {
        "tVTOL2AfterBattery1Slot_To_VTOL2WithBatt",
    },
    "pVTOL2Battery2WithBatt": {
        "tVTOL2Battery2WithBatt_To_VTOL2InBattery2Slot",
    },
    "pVTOL2InBattery2Slot": {
        "tVTOL2InBattery2Slot_To_VTOL2AfterBattery2Slot",
    },
    "pVTOL2AfterBattery2Slot": {
        "tVTOL2AfterBattery2Slot_To_VTOL2WithBatt",
    },
}