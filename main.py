import logging
import sys
import threading
from queue import Queue
from PyQt5 import QtWidgets

from config import ROBOT_IP, OPC_ENDPOINT, LOG_PATH, PLC_MANIPULATOR_ADDRESS
from opc_client import OPCUAClientManipulator
from utils import setup_logging
from robot_controller import RobotController
from opc_handler import OPCHandler
from ui_form_wp_gen import MainWindow
from commands import Command, CmdType

from heartbeat import Heartbeat
from watchdog_manager import WatchdogManager


class MainAppClass:
    def __init__(self):
        self.logger = setup_logging(LOG_PATH)
        self.intentional_exit = False

        self.heartbit = Heartbeat(names=['rc', 'opc_server', 'opc_client'], ttl_sec=10.0)

        self.cmd_queue = Queue(maxsize=1000)

        self.RobotController = RobotController(ROBOT_IP, self.cmd_queue,
                                               self.logger,
                                               heartbeat_cb=self.heartbit.beat)
        self.ControllerThread = threading.Thread(
            target=self.RobotController.run,
            name="RobotControllerThread", daemon=True)
        self.ControllerThread.start()
        self.rc_ready = threading.Event()

        self.OpcHandler = OPCHandler(OPC_ENDPOINT, self.RobotController,
                                     self.cmd_queue,
                                     self.logger,
                                     heartbeat_cb=self.heartbit.beat)
        self.OpcThread = threading.Thread(target=self.OpcHandler.start,
                                          name="OPCThread", daemon=True)
        self.OpcThread.start()
        self.opc_ready = threading.Event()

        self.OpcClientManipulator = OPCUAClientManipulator(PLC_MANIPULATOR_ADDRESS,
                                                           self.RobotController,
                                                           self.logger)
        self.OpcClientManipulator.start_in_thread()

        self.watchdog = WatchdogManager(self.heartbit, interval_sec=5.0,
                                        logger=self.logger)
        self.watchdog.start()

        self.App = QtWidgets.QApplication([])

        self.Form = MainWindow(self.RobotController, self.cmd_queue, self.OpcHandler)
        self.Form.closeEvent = self.on_close
        self.Form.show()

        self.watchdog.set_ready()
        self.logger.info("READY=1 sent")

    def hide_form(self, event):
        self.Form.hide()
        event.ignore()

    def on_close(self, event):
        self.watchdog.stop()
        self.intentional_exit = True
        try:
            self.cmd_queue.put_nowait(
                Command(CmdType.SHUTDOWN, {}, source="APP"))
        except Exception:
            pass
        self.OpcHandler.stop()
        self.RobotController.stop()
        self.OpcClientManipulator.stop_from_thread()

        event.accept()

    def run(self):
        self.App.exec_()


def main():
    try:
        app = MainAppClass()
        code = app.run()
        return 0
    except Exception:
        logging.exception("FATAL: unhandled exception")
        return 1


if __name__ == "__main__":
    sys.exit(main())
