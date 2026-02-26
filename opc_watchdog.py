# OPC UA Watchdog: отправляет 0/1 в PLC, читает ответ с другого узла.
# При несоответствии или таймауте — ошибка в логгер.

import asyncio
import threading

from asyncua import Client, ua

from config import PLC_MANIPULATOR_ADDRESS, OPC_WATCHDOG_WRITE_NODE, OPC_WATCHDOG_READ_NODE


class OPCUAWatchdog:
    """
    Watchdog для OPC UA соединения с PLC.

    Каждую секунду:
      - Пишет чередующееся значение (0, 1, 0, 1, ...) в write_node
      - Читает значение из read_node
      - Если прочитанное != отправленному — пишет ошибку в logger

    При обрыве соединения — пытается переподключиться.
    """

    def __init__(
            self,
            endpoint: str,
            write_node: str,
            read_node: str,
            logger,
            interval_sec: float = 1.0,
            timeout_sec: float = 3.0,
            reconnect_delay_sec: float = 5.0,
    ):
        self.endpoint = endpoint
        self.write_node = write_node
        self.read_node = read_node
        self.logger = logger
        self.interval = interval_sec
        self.timeout = timeout_sec
        self.reconnect_delay = reconnect_delay_sec

        self._running = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Запустить watchdog в фоновом потоке."""
        self._running = True
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_until_complete,
            args=(self._run(),),
            daemon=True,
            name="OPCWatchdogThread",
        )
        self._thread.start()
        self.logger.info("OPC UA Watchdog запущен")

    def stop(self) -> None:
        """Остановить watchdog из внешнего потока."""
        self._running = False
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=10)
        self.logger.info("OPC UA Watchdog остановлен")

    async def _run(self) -> None:
        """Основной цикл с автоматическим переподключением."""
        counter = 0

        while self._running:
            try:
                client = Client(url=self.endpoint)
                await asyncio.wait_for(client.connect(), timeout=self.timeout)
                self.logger.info(f"Watchdog: подключён к {self.endpoint}")

                try:
                    counter = await self._watchdog_loop(client, counter)
                finally:
                    try:
                        await client.disconnect()
                    except Exception as err:
                        print(err)

            except asyncio.TimeoutError:
                self.logger.error(
                    f"Watchdog: таймаут подключения к {self.endpoint}"
                )
            except Exception as e:
                self.logger.error(f"Watchdog: ошибка соединения — {e}")

            if self._running:
                self.logger.info(
                    f"Watchdog: повторное подключение через {self.reconnect_delay} с"
                )
                await asyncio.sleep(self.reconnect_delay)

    async def _watchdog_loop(self, client: Client, counter: int) -> int:
        """Цикл пинга. Возвращает текущий счётчик при выходе."""
        w_node = client.get_node(f"ns=4;s={self.write_node}")
        r_node = client.get_node(f"ns=4;s={self.read_node}")

        while self._running:
            value = counter % 2  # 0 → 1 → 0 → ...

            # --- Запись ---
            try:
                await asyncio.wait_for(
                    w_node.set_value(
                        ua.DataValue(ua.Variant(value, ua.VariantType.Int16))
                    ),
                    timeout=self.timeout,
                )
            except asyncio.TimeoutError:
                self.logger.error(
                    f"Watchdog: таймаут записи в {self.write_node} "
                    f"(значение={value})"
                )
                counter += 1
                await asyncio.sleep(self.interval)
                continue
            except Exception as e:
                self.logger.error(f"Watchdog: ошибка записи — {e}")
                raise  # переподключение

            # --- Чтение ---
            try:
                response = await asyncio.wait_for(
                    r_node.read_value(),
                    timeout=self.timeout,
                )
            except asyncio.TimeoutError:
                self.logger.error(
                    f"Watchdog: таймаут чтения из {self.read_node}"
                )
                counter += 1
                await asyncio.sleep(self.interval)
                continue
            except Exception as e:
                self.logger.error(f"Watchdog: ошибка чтения — {e}")
                raise  # переподключение

            # --- Проверка ---
            try:
                received = int(response)
            except (TypeError, ValueError):
                self.logger.error(
                    f"Watchdog: неожиданный тип ответа — {response!r}"
                )
                counter += 1
                await asyncio.sleep(self.interval)
                continue

            if received != value:
                self.logger.error(
                    f"Watchdog: несоответствие! отправлено={value}, "
                    f"получено={received} "
                    f"(узел={self.read_node})"
                )
            else:
                self.logger.debug(f"Watchdog OK: {value} == {received}")

            counter += 1
            await asyncio.sleep(self.interval)

        return counter


def create_watchdog(logger, endpoint: str | None = None) -> OPCUAWatchdog:
    return OPCUAWatchdog(
        endpoint=endpoint or PLC_MANIPULATOR_ADDRESS,
        write_node=OPC_WATCHDOG_WRITE_NODE,
        read_node=OPC_WATCHDOG_READ_NODE,
        logger=logger,
        interval_sec=1.0,
        timeout_sec=3.0,
    )
