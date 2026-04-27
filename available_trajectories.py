available_trajectories = {
    "pHomePosition":
        {"tHomePosition_To_HelicopterModule",
         "tHomePosition_To_Payload",
         "tHomePosition_To_Grippers",
         "tHomePosition_To_Charger",
         "tHomePosition_To_VTOLModule"},  # pHomePosition
    "pHelicopterModule":
        {"tHelicopterModule_To_HomePosition",
         "tHelicopterModule_To_Helicopter1",
         "tHelicopterModule_To_Helicopter2"},  # pHelicopterModule
    "pHelicopter1":
        {"tHelicopter1_To_HelicopterModule",
         "tHelicopter1_To_Helicopter1Payload"},  # pHelicopter1
    "pHelicopter1Payload":
        {"tHelicopter1Payload_To_Helicopter1"},  # pHelicopter1Payload
    "pHelicopter2":
        {"tHelicopter2_To_HelicopterModule",
         "tHelicopter2_To_Helicopter2Payload"},  # pHelicopter2
    "pHelicopter2Payload":
        {"tHelicopter2Payload_To_Helicopter2"},  # pHelicopter2Payload
    "pPayload":
        {"tPayload_To_HomePosition",
         "tPayload_To_Payload1",
         "tPayload_To_Payload2"},  # pPayload
    "pPayload1":
        {"tPayload1_To_Payload"},  # pPayload1
    "pPayload2":
        {"tPayload2_To_Payload"},  # pPayload2
    "pGrippers":
        {"tGrippers_To_HomePosition",
         "tGrippers_To_Grippers1",
         "tGrippers_To_Grippers2"},  # pGrippers
    "pGrippers1":
        {"tGrippers1_To_Grippers"},  # pGrippers1
    "pGrippers2":
        {"tGrippers2_To_Grippers"},  # pGrippers2
    "pCharger":
        {"tCharger_To_HomePosition",
         "tCharger_To_Charger1",
         "tCharger_To_Charger2"},  # pCharger
    "pCharger1":
        {"tCharger1_To_Charger"},  # pCharger1
    "pCharger2":
        {"tCharger2_To_Charger"},  # pCharger2
    "pVTOLModule":
        {"tVTOLModule_To_HomePosition",
         "tVTOLModule_To_VTOL1",
         "tVTOLModule_To_VTOL2"},  # pVTOLModule
    "pVTOL1":
        {"tVTOL1_To_VTOLModule",
         "tVTOL1_To_VTOL1Payload",
         "tVTOL1_To_VTOL1Battery"},  # pVTOL1
    "pVTOL1Payload":
        {"tVTOL1Payload_To_VTOL1"},  # pVTOL1Payload
    "pVTOL1Battery":
        {"tVTOL1Battery_To_VTOL1"},  # pVTOL1Battery
    "pVTOL2":
        {"tVTOL2_To_VTOLModule", "tVTOL2_To_VTOL2Battery", "tVTOL2Battery_To_VTOL2"}, # pVTOL2
    "pVTOL2Battery":
        {"tVTOL2Battery_To_VTOL2"}, # pVTOL2Battery
}
