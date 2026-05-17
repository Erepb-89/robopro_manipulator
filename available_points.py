available_points = {
    "pHomePosition":
        ["pHelicopterModule",
         "pPayload",
         "pGrippers",
         "pCharger",
         "pVTOLModule",
         "pHomePositionWithSomeCargo",
         ],  # pHomePosition
    "pHelicopterModule":
        ["pHomePosition",
         "pHelicopter1",
         "pHelicopter2",
         "pHelicopterModuleWithSomeCargo",
         ],  # pHelicopterModule

    "pHelicopterModuleWithSomeCargo":
        ["pHomePositionWithSomeCargo",
         "pHelicopter1WithPL",
         "pHelicopter2WithSomeCargo",
         "pHelicopterModule",
         ],  # pHelicopterModuleWithSomeCargo

    "pHelicopter1WithPL":
        ["pHelicopterModuleWithSomeCargo",
         "pHelicopter1PayloadWithPL"],  # pHelicopter1WithPL

    "pHelicopter2":
        ["pHelicopterModule",
         "pHelicopter2Payload,"
         "pHelicopter2Battery1,"
         "pHelicopter2Battery2,"],  # pHelicopter2
    "pHelicopter2Payload":
        ["pHelicopter2"],  # pHelicopter2Payload
    "pHelicopter2Battery1":
        ["pHelicopter2",
         "pHelicopter2InsideBattery1Slot",
         "pHelicopter2BeforeBattery1Slot"],  # pHelicopter2Battery1
    "pHelicopter2InsideBattery1Slot":
        ["pHelicopter2Battery1",
         "pHelicopter2BeforeBattery1Slot"],  # pHelicopter2InsideBattery1Slot
    "pHelicopter2BeforeBattery1Slot":
        ["pHelicopter2WithSomeCargo",
         "pHelicopter2",
         ],  # pHelicopter2BeforeBattery1Slot

    "pHelicopter2Battery2":
        ["pHelicopter2",
         "pHelicopter2InsideBattery2Slot",
         "pHelicopter2BeforeBattery2Slot"],  # pHelicopter2Battery2
    "pHelicopter2InsideBattery2Slot":
        ["pHelicopter2Battery2",
         "pHelicopter2BeforeBattery2Slot"],  # pHelicopter2InsideBattery2Slot
    "pHelicopter2BeforeBattery2Slot":
        ["pHelicopter2WithSomeCargo"
         "pHelicopter2",
         ],  # pHelicopter2BeforeBattery2Slot

    "pHelicopter2WithSomeCargo":
        ["pHelicopter2WithBatt1",
         "pHelicopter2WithBatt2",
         "pHelicopter2PayloadWithPL",
         "pHelicopter2AfterSlot1",
         "pHelicopter2AfterSlot2"],  # pHelicopter2WithSomeCargo
    "pHelicopter2WithBatt1":
        ["pHelicopter2WithSomeCargo",
         "pHelicopter2InSlot1",
         "pHelicopter2AfterSlot1"],  # pHelicopter2WithBatt1
    "pHelicopter2InSlot1":
        ["pHelicopter2WithBatt1",
         "pHelicopter2AfterSlot1"],  # pHelicopter2InSlot1
    "pHelicopter2AfterSlot1":
        ["pHelicopter2WithSomeCargo"],  # pHelicopter2AfterSlot1

    "pHelicopter2WithBatt2":
        ["pHelicopter2WithSomeCargo",
         "pHelicopter2InSlot2",
         "pHelicopter2AfterSlot2"],  # pHelicopter2WithBatt2
    "pHelicopter2InSlot2":
        ["pHelicopter2WithBatt2",
         "pHelicopter2AfterSlot2"],  # pHelicopter2InSlot2
    "pHelicopter2AfterSlot2":
        ["pHelicopter2WithSomeCargo"],  # pHelicopter2AfterSlot2

    "pHelicopter1":
        ["pHelicopterModule",
         "pHelicopter1Payload"],  # pHelicopter1
    "pHelicopter1InsideSlot":
        ["pHelicopter1Payload",
         "pHelicopter1BeforeSlot"],  # pHelicopter1InsideSlot
    "pHelicopter1BeforeSlot":
        ["pHelicopter1WithPL",
         "pHelicopter1",
         ],  # pHelicopter1BeforeSlot

    "pHelicopter1Payload":
        ["pHelicopter1",
         "pHelicopter1InsideSlot",
         "pHelicopter1BeforeSlot"],  # pHelicopter1Payload

    "pHelicopter1PayloadWithPL":
        ["pHelicopter1",
         "pHelicopter1InSlot",
         "pHelicopter1AfterSlot"],  # pHelicopter1PayloadWithPL
    "pHelicopter1InSlot":
        ["pHelicopter1PayloadWithPL",
         "pHelicopter1AfterSlot"],  # pHelicopter1InSlot
    "pHelicopter1AfterSlot":
        ["pHelicopter1"],  # pHelicopter1AfterSlot

    "pPayload":
        ["pHomePosition",
         "pPayload1",
         "pPayload2",
         "pPayloadWithPL",
         ],  # pPayload

    "pPayload1":
        ["pPayload",
         "pPayload1InsideSlot",
         "pPayload1BeforeSlot"],  # pPayload1
    "pPayload1InsideSlot":
        ["pPayload1",
         "pPayload1BeforeSlot"],  # pPayload1InsideSlot
    "pPayload1BeforeSlot":
        ["pPayload1"],  # pPayload1BeforeSlot

    "pPayload1WithPL":
        ["pPayload",
         "pPayload1InSlot",
         "pPayload1AfterSlot"],  # pPayload1WithPL
    "pPayload1InSlot":
        ["pPayload1WithPL",
         "pPayload1AfterSlot"],  # pPayload1InSlot
    "pPayload1AfterSlot":
        ["pPayload"],  # pPayload1AfterSlot

    "pPayload2":
        ["pPayload",
         "pPayload2InsideSlot",
         "pPayload2BeforeSlot"],  # pPayload2
    "pPayload2InsideSlot":
        ["pPayload2",
         "pPayload2BeforeSlot"],  # pPayload2InsideSlot
    "pPayload2BeforeSlot":
        ["pPayload2"],  # pPayload2BeforeSlot

    "pPayload2WithPL":
        ["pPayload",
         "pPayload2InSlot",
         "pPayload2AfterSlot"],  # pPayload2WithPL
    "pPayload2InSlot":
        ["pPayload2WithPL",
         "pPayload2AfterSlot"],  # pPayload2InSlot
    "pPayload2AfterSlot":
        ["pPayload"],  # pPayload2AfterSlot

    "pGrippers":
        ["pHomePosition",
         "pGrippers1",
         "pGrippers2",
         "pGrippersWithGrip",
         ],  # pGrippers

    "pGrippers1":
        ["pGrippers",
         "pGrippers1InsideSlot",
         "pGrippers1BeforeSlot"],  # pGrippers1
    "pGrippers1InsideSlot":
        ["pGrippers1",
         "pGrippers1BeforeSlot"],  # pGrippers1InsideSlot
    "pGrippers1BeforeSlot":
        ["pGrippers"],  # pGrippers1BeforeSlot

    "pGrippersWithGrip":
        ["pHomePositionWithSomeCargo",
         "pGrippers1WithGrip",
         "pGrippers2WithGrip",
         "pGrippers",
         ],  # pGrippersWithGrip
    "pGrippers1WithGrip":
        ["pGrippersWithGrip",
         "pGrippers1InSlot",
         "pGrippers1AfterSlot"],  # pGrippers1WithGrip
    "pGrippers1InSlot":
        ["pGrippers1WithGrip",
         "pGrippers1AfterSlot"],  # pGrippers1InSlot
    "pGrippers1AfterSlot":
        ["pGrippersWithGrip"],  # pGrippers1AfterSlot

    "pGrippers2":
        ["pGrippersWithGrip",
         "pGrippers2InsideSlot",
         "pGrippers2BeforeSlot"],  # pGrippers2
    "pGrippers2InsideSlot":
        ["pGrippers2",
         "pGrippers2BeforeSlot"],  # pGrippers2InsideSlot
    "pGrippers2BeforeSlot":
        ["pGrippersWithGrip",
         "pGrippers",
         ],  # pGrippers2BeforeSlot

    "pGrippers2WithGrip":
        ["pGrippers",
         "pGrippers2InSlot",
         "pGrippers2AfterSlot"],  # pGrippers2WithGrip
    "pGrippers2InSlot":
        ["pGrippers2WithGrip",
         "pGrippers2AfterSlot"],  # pGrippers2InSlot
    "pGrippers2AfterSlot":
        ["pGrippers"],  # pGrippers2AfterSlot

    "pCharger":
        ["pHomePosition",
         "pVTOL2Charger1",
         "pVTOL2Charger2",
         "pHelicopter2Charger1",
         "pChargerWithBatt",
         ],  # pCharger

    "pChargerWithBatt":
        ["pHomePositionWithSomeCargo",
         "pVTOL2Charger1WithBatt",
         "pVTOL2Charger2WithBatt",
         "pHelicopter2Charger1WithBatt",
         "pCharger",
         ],  # pChargerWithBatt

    "pVTOL2Charger1WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger1Slot1WithBatt",
         "pVTOL2Charger1Slot2WithBatt"],  # pVTOL2Charger1WithBatt
    "pVTOL2Charger2WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger2Slot1WithBatt",
         "pVTOL2Charger2Slot2WithBatt"],  # pVTOL2Charger2WithBatt
    "pHelicopter2Charger1WithBatt":
        ["pChargerWithBatt",
         "pHelicopter2Charger1Slot1WithBatt",
         "pHelicopter2Charger1Slot2WithBatt"],  # pHelicopter2Charger1WithBatt

    "pVTOL2Charger1":
        ["pCharger",
         "pVTOL2Charger1Slot1",
         "pVTOL2Charger1Slot2",
         ],  # pVTOL2Charger1

    "pVTOL2Charger1Slot1":
        ["pVTOL2Charger1",
         "pVTOL2Charger1InsideSlot1",
         "pVTOL2Charger1BeforeSlot1"],  # pVTOL2Charger1Slot1
    "pVTOL2Charger1InsideSlot1":
        ["pVTOL2Charger1Slot1",
         "pVTOL2Charger1BeforeSlot1_pre",
         "pVTOL2Charger1BeforeSlot1"],  # pVTOL2Charger1InsideSlot1
    "pVTOL2Charger1BeforeSlot1":
        ["pVTOL2Charger1WithBatt",
         "pVTOL2Charger1",
         ],  # pVTOL2Charger1BeforeSlot1

    "pVTOL2Charger1Slot1WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger1InSlot1",
         "pVTOL2Charger1AfterSlot1"],  # pVTOL2Charger1Slot1WithBatt
    "pVTOL2Charger1InSlot1":
        ["pVTOL2Charger1Slot1WithBatt",
         "pVTOL2Charger1AfterSlot1"],  # pVTOL2Charger1InSlot1
    "pVTOL2Charger1AfterSlot1":
        ["pVTOL2Charger1WithBatt"],  # pVTOL2Charger1AfterSlot1

    "pVTOL2Charger1Slot2":
        ["pVTOL2Charger1",
         "pVTOL2Charger1InsideSlot2",
         "pVTOL2Charger1BeforeSlot2"],  # pVTOL2Charger1Slot2
    "pVTOL2Charger1InsideSlot2":
        ["pVTOL2Charger1Slot2",
         "pVTOL2Charger1BeforeSlot2"],  # pVTOL2Charger1InsideSlot2
    "pVTOL2Charger1BeforeSlot2":
        ["pVTOL2Charger1WithBatt",
         "pVTOL2Charger1",
         ],  # pVTOL2Charger1BeforeSlot2

    "pVTOL2Charger1Slot2WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger1InSlot2",
         "pVTOL2Charger1AfterSlot2"],  # pVTOL2Charger1Slot2WithBatt
    "pVTOL2Charger1InSlot2":
        ["pVTOL2Charger1Slot2WithBatt",
         "pVTOL2Charger1AfterSlot2"],  # pVTOL2Charger1InSlot2
    "pVTOL2Charger1AfterSlot2":
        ["pVTOL2Charger1WithBatt"],  # pVTOL2Charger1AfterSlot2

    "pVTOL2Charger2":
        ["pCharger",
         "pVTOL2Charger2Slot1",
         "pVTOL2Charger2Slot2"],  # pVTOL2Charger2

    "pVTOL2Charger2Slot1":
        ["pVTOL2Charger2",
         "pVTOL2Charger2InsideSlot1",
         "pVTOL2Charger2BeforeSlot1"],  # pVTOL2Charger2Slot1
    "pVTOL2Charger2InsideSlot1":
        ["pVTOL2Charger2Slot1",
         "pVTOL2Charger2BeforeSlot1"],  # pVTOL2Charger2InsideSlot1
    "pVTOL2Charger2BeforeSlot1":
        ["pVTOL2Charger2WithBatt",
         "pVTOL2Charger2",
         ],  # pVTOL2Charger2BeforeSlot1

    "pVTOL2Charger2Slot1WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger2InSlot1",
         "pVTOL2Charger2AfterSlot1"],  # pVTOL2Charger2Slot1WithBatt
    "pVTOL2Charger2InSlot1":
        ["pVTOL2Charger2Slot1WithBatt",
         "pVTOL2Charger2AfterSlot1"],  # pVTOL2Charger2InSlot1
    "pVTOL2Charger2AfterSlot1":
        ["pChargerWithBatt"],  # pVTOL2Charger2AfterSlot1

    "pVTOL2Charger2Slot2":
        ["pVTOL2Charger2",
         "pVTOL2Charger2InsideSlot2",
         "pVTOL2Charger2BeforeSlot2"],  # pVTOL2Charger2Slot2
    "pVTOL2Charger2InsideSlot2":
        ["pVTOL2Charger2Slot2",
         "pVTOL2Charger2BeforeSlot2"],  # pVTOL2Charger2InsideSlot2
    "pVTOL2Charger2BeforeSlot2":
        ["pVTOL2Charger2WithBatt",
         "pVTOL2Charger2",
         ],  # pVTOL2Charger2BeforeSlot2

    "pVTOL2Charger2Slot2WithBatt":
        ["pChargerWithBatt",
         "pVTOL2Charger2InSlot2",
         "pVTOL2Charger2AfterSlot2"],  # pVTOL2Charger2Slot2WithBatt
    "pVTOL2Charger2InSlot2":
        ["pVTOL2Charger2Slot2WithBatt",
         "pVTOL2Charger2AfterSlot2"],  # pVTOL2Charger2InSlot2
    "pVTOL2Charger2AfterSlot2":
        ["pChargerWithBatt"],  # pVTOL2Charger2AfterSlot2

    "pHelicopter2Charger1":
        ["pCharger",
         "pHelicopter2Charger1Slot1",
         "pHelicopter2Charger1Slot2",
         "pHelicopter2Charger1BeforeSlot1"],  # pHelicopter2Charger1

    "pHelicopter2Charger1Slot1":
        ["pHelicopter2Charger1",
         "pHelicopter2Charger1InsideSlot1",
         "pHelicopter2Charger1BeforeSlot1"],  # pHelicopter2Charger1Slot1
    "pHelicopter2Charger1InsideSlot1":
        ["pHelicopter2Charger1Slot1",
         "pHelicopter2Charger1BeforeSlot1"],  # pHelicopter2Charger1InsideSlot1
    "pHelicopter2Charger1BeforeSlot1":
        ["pHelicopter2Charger1WithBatt",
         "pHelicopter2Charger1",
         ],  # pHelicopter2Charger1BeforeSlot1

    "pHelicopter2Charger1Slot1WithBatt":
        ["pChargerWithBatt",
         "pHelicopter2Charger1InSlot1",
         "pHelicopter2Charger1AfterSlot1"],  # pHelicopter2Charger1Slot1WithBatt
    "pHelicopter2Charger1InSlot1":
        ["pHelicopter2Charger1Slot1WithBatt",
         "pHelicopter2Charger1AfterSlot1"],  # pHelicopter2Charger1InSlot1
    "pHelicopter2Charger1AfterSlot1":
        ["pChargerWithBatt"],  # pHelicopter2Charger1AfterSlot1

    "pHelicopter2Charger1Slot2":
        ["pHelicopter2Charger1",
         "pHelicopter2Charger1InsideSlot2",
         "pHelicopter2Charger1BeforeSlot2"],  # pHelicopter2Charger1Slot2
    "pHelicopter2Charger1InsideSlot2":
        ["pHelicopter2Charger1Slot2",
         "pHelicopter2Charger1BeforeSlot2"],  # pHelicopter2Charger1InsideSlot2
    "pHelicopter2Charger1BeforeSlot2":
        ["pHelicopter2Charger1WithBatt",
         "pHelicopter2Charger1", ],  # pHelicopter2Charger1BeforeSlot2

    "pHelicopter2Charger1Slot2WithBatt":
        ["pChargerWithBatt",
         "pHelicopter2Charger1InSlot2",
         "pHelicopter2Charger1AfterSlot2"],  # pHelicopter2Charger1Slot2WithBatt
    "pHelicopter2Charger1InSlot2":
        ["pHelicopter2Charger1Slot2WithBatt",
         "pHelicopter2Charger1AfterSlot2"],  # pHelicopter2Charger1InSlot2
    "pHelicopter2Charger1AfterSlot2":
        ["pChargerWithBatt"],  # pHelicopter2Charger1AfterSlot2

    "pVTOLModule":
        ["pHomePosition",
         "pVTOL1",
         "pVTOL2",
         "pVTOLModuleWithSomeCargo",
         ],  # pVTOLModule

    "pVTOLModuleWithSomeCargo":
        ["pHomePositionWithSomeCargo",
         "pVTOL1WithSomeCargo",
         "pVTOL2WithBatt",
         "pVTOLModule",
         ],  # pVTOLModuleWithSomeCargo

    "pVTOL1":
        ["pVTOLModule",
         "pVTOL1Payload",
         "pVTOL1Battery"],  # pVTOL1
    "pVTOL1Battery":
        ["pVTOL1",
         "pVTOL1InsideBatterySlot",
         "pVTOL1BeforeBatterySlot"],  # pVTOL1Battery
    "pVTOL1InsideBatterySlot":
        ["pVTOL1Battery",
         "pVTOL1BeforeBatterySlot"],  # pVTOL1InsideBatterySlot
    "pVTOL1BeforeBatterySlot":
        ["pVTOL1WithSomeCargo",
         "pVTOL1",
         ],  # pVTOL1BeforeBatterySlot

    "pVTOL1WithSomeCargo":
        ["pVTOLModuleWithSomeCargo",
         "pVTOL1BatteryWithBatt",
         "pVTOL1AfterBatterySlot"],  # pVTOL1WithSomeCargo

    "pVTOL1BatteryWithBatt":
        ["pVTOL1WithSomeCargo",
         "pVTOL1InBatterySlot",
         "pVTOL1AfterBatterySlot"],  # pVTOL1BatteryWithBatt
    "pVTOL1InBatterySlot":
        ["pVTOL1BatteryWithBatt",
         "pVTOL1AfterBatterySlot"],  # pVTOL1InBatterySlot
    "pVTOL1AfterBatterySlot":
        ["pVTOL1WithSomeCargo"],  # pVTOL1AfterBatterySlot

    """
        pVTOL1PayloadWithPL
        pVTOL1InPayloadSlot
        pVTOL1AfterPayloadSlot
    """
    "pVTOL2":
        ["pVTOLModule",
         "pVTOL2Battery1",
         "pVTOL2Battery2"],  # pVTOL2

    "pVTOL2WithBatt":
        ["pVTOLModuleWithSomeCargo",
         "pVTOL2Battery1WithBatt",
         "pVTOL2Battery2WithBatt",
         "pVTOL2AfterBattery1Slot",
         "pVTOL2AfterBattery2Slot"],  # pVTOL2WithBatt

    "pVTOL2Battery1":
        ["pVTOL2WithBatt",
         "pVTOL2InsideBattery1Slot",
         "pVTOL2BeforeBattery1Slot"],  # pVTOL2Battery1
    "pVTOL2InsideBattery1Slot":
        ["pVTOL2Battery1",
         "pVTOL2BeforeBattery1Slot"],  # pVTOL2InsideBattery1Slot
    "pVTOL2BeforeBattery1Slot":
        ["pVTOL2WithBatt",
         "pVTOL2",
         ],  # pVTOL2BeforeBattery1Slot

    "pVTOL2Battery1WithBatt":
        ["pVTOL2WithBatt",
         "pVTOL2InBattery1Slot",
         "pVTOL2AfterBattery1Slot"],  # pVTOL2Battery1WithBatt
    "pVTOL2InBattery1Slot":
        ["pVTOL2Battery1WithBatt",
         "pVTOL2AfterBattery1Slot"],  # pVTOL2InBattery1Slot
    "pVTOL2AfterBattery1Slot":
        ["pVTOL2WithBatt"],  # pVTOL2AfterBattery1Slot

    "pVTOL2Battery2WithBatt":
        ["pVTOL2WithBatt",
         "pVTOL2InBattery2Slot",
         "pVTOL2AfterBattery2Slot"],  # pVTOL2Battery2WithBatt
    "pVTOL2InBattery2Slot":
        ["pVTOL2Battery2WithBatt",
         "pVTOL2AfterBattery2Slot"],  # pVTOL2InBattery2Slot
    "pVTOL2AfterBattery2Slot":
        ["pVTOL2WithBatt"],  # pVTOL2AfterBattery2Slot

    "pVTOL2Battery2":
        ["pVTOL2",
         "pVTOL2InsideBattery2Slot",
         "pVTOL2BeforeBattery2Slot"],  # pVTOL2Battery2
    "pVTOL2InsideBattery2Slot":
        ["pVTOL2Battery2",
         "pVTOL2BeforeBattery2Slot"],  # pVTOL2InsideBattery2Slot
    "pVTOL2BeforeBattery2Slot":
        ["pVTOL2WithBatt",
         "pVTOL2",
         ],  # pVTOL2BeforeBattery2Slot

    "pHomePositionWithSomeCargo":
        ["pHelicopterModuleWithSomeCargo",
         "pPayloadWithPL",
         "pChargerWithBatt",
         "pVTOLModuleWithSomeCargo",
         "pGrippersWithGrip",
         "pHomePosition",
         ],  # pHomePositionWithSomeCargo
    "pPayloadWithPL":
        ["pHomePositionWithSomeCargo",
         "pPayload1WithPL",
         "pPayload2WithPL",
         "pPayload",
         ],  # pPayloadWithPL
}
