import time
import threading
from sdnotify import SystemdNotifier


class WatchdogManager(threading.Thread):
    def __init__(self, heartbit, interval_sec=5.0, logger=None):
        super().__init__(daemon=True, name="WatchdogThread")
        self.heartbit = heartbit
        self.interval = interval_sec
        self.log = logger
        self.stop_event = threading.Event()
        self.notifier = SystemdNotifier()
        self.ready_sent = False

    def set_ready(self):
        try:
            self.notifier.notify("READY=1")
            self.ready_sent = True
        except Exception as e:
            if self.log: self.log.debug(f"sdnotify READY errr: {e}")

    def run(self):
        while not self.stop_event.is_set():
            try:
                statuses = self.heartbit.state()
                all_is_ok = all(statuses.values()) if statuses else True
                status_line = "; ".join(
                    f"{k}={'ok' if v else 'stale'}" for k, v in
                    statuses.items())
                if status_line:
                    self.notifier.notify(f"STATUS={status_line}")
                    self.log.info(f"[WD]={status_line}")

                if all_is_ok:
                    self.notifier.notify("WATCHDOG=1")

            except Exception as e:
                if self.log:
                    self.log.debug(f"Watchdog loop error: {e}")
            time.sleep(self.interval)

    def stop(self):
        self.stop_event.set()
        try:
            self.notifier.notify("STOPPING=1")
        except Exception:
            pass
