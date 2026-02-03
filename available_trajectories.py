available_trajectories = {
    "pHomePosition":
        {"tHomePosition_To_HelicopterModule",
         "tHomePosition_To_Load",
         "tHomePosition_To_Grippers",
         "tHomePosition_To_Charger",
         "tHomePosition_To_VTOLModule"},  # pHomePosition
    "pHelicopterModule":
        {"tHelicopterModule_To_HomePosition",
         "tHelicopterModule_To_Helicopter1",
         "tHelicopterModule_To_Helicopter2"},  # pHelicopterModule
    "pHelicopter1":
        {"tHelicopter1_To_HelicopterModule",
         "tHelicopter1_To_Helicopter1Load"},  # pHelicopter1
    "pHelicopter1Load":
        {"tHelicopter1Load_To_Helicopter1"},  # pHelicopter1Load
    "pHelicopter2":
        {"tHelicopter2_To_HelicopterModule",
         "tHelicopter2_To_Helicopter2Load"},  # pHelicopter2
    "pHelicopter2Load":
        {"tHelicopter2Load_To_Helicopter2"},  # pHelicopter2Load
    "pLoad":
        {"tLoad_To_HomePosition",
         "tLoad_To_Load1",
         "tLoad_To_Load2"},  # pLoad
    "pLoad1":
        {"tLoad1_To_Load"},  # pLoad1
    "pLoad2":
        {"tLoad2_To_Load"},  # pLoad2
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
         "tVTOL1_To_VTOL1Load",
         "tVTOL1_To_VTOL1Battery"},  # pVTOL1
    "pVTOL1Load":
        {"tVTOL1Load_To_VTOL1"},  # pVTOL1Load
    "pVTOL1Battery":
        {"tVTOL1Battery_To_VTOL1"},  # pVTOL1Battery
    "pVTOL2":
        {"tVTOL2_To_VTOLModule", "tVTOL2_To_VTOL2Battery"},  # pVTOL2
    "pVTOL2Battery":
        {"tVTOL2Battery_To_VTOL2"},  # pVTOL2Battery
}
