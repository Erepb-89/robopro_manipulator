from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# Пути к данным
POINTS_PATH = BASE_DIR / "points.json"
TRAJ_PATH = BASE_DIR / "trajectories.json"

# Параметры подключения
ROBOT_IP = "127.0.0.1"
OPC_ENDPOINT = "opc.tcp://0.0.0.0:4840"

# IO и прочее
NUM_DIGITAL_IO = 24
GRIPPER_DO_INDEX = 0
LOG_PATH = BASE_DIR / "robopro.log"