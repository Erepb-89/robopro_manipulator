# OPC UA клиент для записи значений в переменные сервера.

import asyncio
import threading
from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum

from asyncua import Client, ua

from commands import Command, CmdType
from config import HELICOPTER_MODULE, VTOL_MODULE, LOAD_STORAGE, GRIPPERS_STORAGE, \
    CHARGER_H, CHARGER_V, HOME_POSITION, ENABLE_MOVE_TO_MODULE_H, ENABLE_MOVE_TO_MODULE_V, \
    ENABLE_MOVE_TO_PAYLOAD_STORAGE, ENABLE_MOVE_TO_GRIPPERS_STORAGE, ENABLE_MOVE_TO_CHARGER_H, \
    ENABLE_MOVE_TO_CHARGER_V, ENABLE_MOVE_TO_HOME, PLC_MANIPULATOR_ADDRESS, X_POSITION_MODULE_H, \
    X_POSITION_MODULE_V, X_POSITION_CHARGER_H, X_POSITION_CHARGER_V, X_POSITION_LOAD, \
    X_POSITION_GRIPPER_STORAGE, X_POSITION_HAS_ZEROED, X_POSITION_POWERED, X_POSITION_ALARM, OPC_CLIENT_TIME, \
    H_TABLE_HATCH_OPENED, H_TABLE_HATCH_CLOSED, H_TABLE_HATCH_ALARM, V_TABLE_HATCH_OPENED, V_TABLE_HATCH_CLOSED, \
    V_TABLE_HATCH_ALARM, Y_POSITION_MODULE_H, Y_POSITION_MODULE_V, Y_POSITION_CHARGER_H, Y_POSITION_CHARGER_V, \
    Y_POSITION_LOAD, Y_POSITION_GRIPPER_STORAGE, Y_POSITION_HAS_ZEROED, Y_POSITION_POWERED, Y_POSITION_ALARM, \
    PLC_CMD_POWER_ON, PLC_CMD_FREE_DRIVE, PLC_CMD_GRIPPER, PLC_CMD_FIND_NEAREST, GRIPPER_DO_INDEX, \
    PLC_CMD_SHIFT_GRIPPER, SHIFT_GRIPPER_DO_INDEX, PLC_CMD_TRAJECTORY, PLC_CMD_ACTION, H_TABLE_LIFT_POS_TOP, \
    H_TABLE_LIFT_POS_BOTTOM, H_TABLE_LIFT_ALARM, V_TABLE_LIFT_POS_TOP, V_TABLE_LIFT_POS_BOTTOM, V_TABLE_LIFT_ALARM, \
    H_BOX_LIFT_POS_TOP, H_BOX_LIFT_POS_BOTTOM, H_BOX_LIFT_ALARM


@dataclass
class ManipulatorPoints:
    module_h_available: bool = False
    module_v_available: bool = False
    charge_h_available: bool = False
    charge_v_available: bool = False
    pos_load_available: bool = False
    pos_grippers_available: bool = False


@dataclass
class WaypointInfo:
    """Информация о текущей точке"""
    name: str
    is_valid: bool = True


class WaypointType(Enum):
    """Типы точек маршрута для маппинга с OPC узлами"""
    HELICOPTER = (HELICOPTER_MODULE, ENABLE_MOVE_TO_MODULE_H)
    VTOL = (VTOL_MODULE, ENABLE_MOVE_TO_MODULE_V)
    LOAD_STORAGE = (LOAD_STORAGE, ENABLE_MOVE_TO_PAYLOAD_STORAGE)
    GRIPPERS_STORAGE = (GRIPPERS_STORAGE, ENABLE_MOVE_TO_GRIPPERS_STORAGE)
    CHARGER_H = (CHARGER_H, ENABLE_MOVE_TO_CHARGER_H)
    CHARGER_V = (CHARGER_V, ENABLE_MOVE_TO_CHARGER_V)
    HOME = (HOME_POSITION, ENABLE_MOVE_TO_HOME)

    def __init__(self, waypoint_set, opc_node):
        self.waypoint_set = waypoint_set
        self.opc_node = opc_node


class OPCUAClient:
    """Базовый класс OPC клиента"""

    def __init__(self,
                 endpoint: str,
                 cmd_queue,
                 robot_controller,
                 logger):
        self.endpoint = endpoint
        self.cmd_queue = cmd_queue
        self.client = Client(url=self.endpoint)
        self.rc = robot_controller
        self.logger = logger
        self.stop_event = threading.Event()
        self._running = True

    def start_in_thread(self):
        """Запуск клиента в отдельном потоке"""
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_until_complete, args=(self.run(),), daemon=True)
        self._thread.start()

    def stop_from_thread(self):
        """Остановка из основного потока"""
        self._running = False
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self.stop(), self._loop)
        if self._thread:
            self._thread.join(timeout=5)

    async def connect(self):
        """Подключение к серверу"""
        await self.client.connect()
        print(f"Подключено к {self.endpoint}")

    async def disconnect(self):
        """Отключение"""
        await self.client.disconnect()
        print("Клиент отключен")

    async def read_node(self, node_name: str) -> int:
        """Чтение значения узла"""
        try:
            node = self.client.get_node(f"ns=4;s={node_name}")
            value = await node.read_value()
            return value
        except Exception as e:
            self.logger.warning(f"OPC Client can't read data from node: {node_name}. {e}")

    async def write_node(self, node_name, value):
        """Запись значения в узел"""
        try:
            node = self.client.get_node(f"ns=4;s={node_name}")
            await node.set_value(ua.DataValue(ua.Variant(value, ua.VariantType.Int16)))
        except Exception as e:
            self.logger.warning(f"OPC Client can't write data to node: {node_name}. {e}")

    async def stop(self) -> None:
        """
        Остановка клиента
        """
        self._running = False
        try:
            await self.disconnect()
        except Exception as e:
            self.logger.error(f"Ошибка при остановке клиента: {e}")
        self.logger.info("OPC UA клиент остановлен")

    async def _update_opc_nodes(self, current_waypoint: str) -> None:
        """
        Обновить OPC узлы на основе текущей точки маршрута
        """
        for wp_type in WaypointType:
            if current_waypoint in wp_type.waypoint_set:
                await self.write_node(wp_type.opc_node, 1)
                self.logger.debug(f"Активирован узел {wp_type.opc_node}")
            else:
                await self.write_node(wp_type.opc_node, 0)

    async def _get_current_waypoint(self) -> WaypointInfo:
        """
        Получить информацию о текущей точке
        """
        try:
            waypoint_data = self.rc.find_nearest_waypoint()
            waypoint_name = waypoint_data.get("waypoint")
            return WaypointInfo(name=waypoint_name, is_valid=True)
        except Exception as e:
            self.logger.error(f"Ошибка получения текущей точки: {e}")
            return WaypointInfo(name="", is_valid=False)

    async def run(self):
        """
        Основной цикл работы клиента
        """
        self._running = True
        self.logger.info("Запуск OPC UA клиента")
        try:
            await self.connect()

            while self._running:
                await self.read_nodes()

                self.check_position()

                await self.get_wp_info_and_update_opc_data()

                await self.handle_plc_commands()

                await asyncio.sleep(OPC_CLIENT_TIME)  # задержка для снижения нагрузки
        except asyncio.CancelledError:
            self.logger.info("Получен сигнал остановки клиента")
        except Exception as e:
            self.logger.error(f"Критическая ошибка в основном цикле клиента: {e}")
        finally:
            await self.stop()

    @abstractmethod
    async def get_wp_info_and_update_opc_data(self):
        """Узнать текущую позицию и обновить данные в OPC"""
        pass

    @abstractmethod
    async def read_nodes(self):
        """Чтение всех узлов сервера"""
        pass

    @abstractmethod
    def check_position(self):
        """Проверка позиции манипулятора по осям"""
        pass

    @abstractmethod
    def handle_plc_commands(self):
        """Чтение командных узлов с PLC и пересылка в cmd_queue"""
        pass

    async def convert(self, node):
        try:
            return bool(await self.read_node(node))
        except Exception as e:
            print(e)


class OPCUAClientManipulator(OPCUAClient):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._plc_power_prev = 0
        self._plc_free_drive_prev = 0
        self._plc_gripper_prev = 0
        self._plc_shift_gripper_prev = 0
        self._plc_traj_prev = 0
        self._plc_action_prev = 0

    async def handle_plc_commands(self) -> None:
        """Чтение командных узлов с PLC и пересылка в cmd_queue"""
        await self.handle_plc_simple_command(PLC_CMD_POWER_ON,
                                             self._plc_power_prev,
                                             CmdType.POWER,
                                             'POWER')

        await self.handle_plc_simple_command(PLC_CMD_FREE_DRIVE,
                                             self._plc_free_drive_prev,
                                             CmdType.FREE_DRIVE,
                                             'FREE_DRIVE')

        await self.handle_plc_simple_command(PLC_CMD_FIND_NEAREST,
                                             self._plc_free_drive_prev,
                                             CmdType.FIND_NEAREST,
                                             'FIND_NEAREST')

        await self.handle_gripper_command(PLC_CMD_GRIPPER,
                                          self._plc_gripper_prev,
                                          GRIPPER_DO_INDEX,
                                          CmdType.IO_SET,
                                          'FREE_DRIVE')

        await self.handle_gripper_command(PLC_CMD_SHIFT_GRIPPER,
                                          self._plc_shift_gripper_prev,
                                          SHIFT_GRIPPER_DO_INDEX,
                                          CmdType.IO_SET,
                                          'FREE_DRIVE')

        await self.handle_traj_cmd(PLC_CMD_TRAJECTORY,
                                   self._plc_traj_prev,
                                   CmdType.EXECUTE_TRAJECTORY,
                                   'EXECUTE_TRAJECTORY')

        await self.handle_traj_cmd(PLC_CMD_ACTION,
                                   self._plc_action_prev,
                                   CmdType.EXECUTE_ACTION,
                                   'EXECUTE_ACTION')

    async def handle_traj_cmd(self,
                              node_name: str,
                              prev_val: int,
                              cmd_type: CmdType,
                              cmd_text: str) -> None:
        """Обработка Траектории или Action с PLC"""
        try:
            plc_cmd = await self.read_node(node_name)
            if prev_val != plc_cmd:
                plc_cmd = int(plc_cmd)
                if plc_cmd == 0:
                    self.cmd_queue.put(
                        Command(CmdType.STOP_MOVE, {},
                                source="OPC"))
                elif plc_cmd in range(1, 100):
                    self.cmd_queue.put(Command(cmd_type,
                                               {'num': int(plc_cmd)},
                                               source="PLC"))
                    self.logger.info(f"PLC command {cmd_text}: {plc_cmd}")
                prev_val = plc_cmd
        except Exception as e:
            self.logger.error(f"Error reading PLC {cmd_text} command: {e}")

    async def handle_plc_simple_command(self,
                                        node_name: str,
                                        prev_val: int,
                                        cmd_type: CmdType,
                                        cmd_text: str) -> None:
        """Обработка простой команды 0/1 с PLC"""
        try:
            plc_cmd = await self.read_node(node_name)
            if plc_cmd is not None:
                plc_cmd = int(plc_cmd)
                if prev_val != plc_cmd:
                    self.cmd_queue.put(Command(cmd_type,
                                               {'state': plc_cmd},
                                               source="PLC"))
                    self.logger.info(f"PLC command {cmd_text}: {plc_cmd}")
                prev_val = plc_cmd
        except Exception as e:
            self.logger.error(f"Error reading PLC {cmd_text} command: {e}")

    async def handle_gripper_command(self,
                                     node_name: str,
                                     prev_val: int,
                                     grip_index: int,
                                     cmd_type: CmdType,
                                     cmd_text: str) -> None:
        """Обработка команды действия с грипперами с PLC"""
        try:
            plc_cmd = await self.read_node(node_name)
            if plc_cmd is not None:
                plc_cmd = int(plc_cmd)
                if prev_val != plc_cmd:
                    self.cmd_queue.put(Command(cmd_type,
                                               {'index': grip_index,
                                                'value': bool(plc_cmd)},
                                               source="PLC"))
                    self.logger.info(f"PLC command {cmd_text}: {plc_cmd}")
                prev_val = plc_cmd
        except Exception as e:
            self.logger.error(f"Error reading PLC {cmd_text} command: {e}")

    async def read_nodes(self):
        """Чтение всех узлов сервера"""
        self.x_pos_module_h = await self.convert(X_POSITION_MODULE_H)
        self.x_pos_module_v = await self.convert(X_POSITION_MODULE_V)
        self.x_pos_charge_h = await self.convert(X_POSITION_CHARGER_H)
        self.x_pos_charge_v = await self.convert(X_POSITION_CHARGER_V)
        self.x_pos_load = await self.convert(X_POSITION_LOAD)
        self.x_pos_gripper_storage = await self.convert(X_POSITION_GRIPPER_STORAGE)
        self.x_pos_has_zeroed = await self.convert(X_POSITION_HAS_ZEROED)
        self.x_pos_powered = await self.convert(X_POSITION_POWERED)
        self.x_pos_alarm = await self.convert(X_POSITION_ALARM)
        self.y_pos_module_h = await self.convert(Y_POSITION_MODULE_H)
        self.y_pos_module_v = await self.convert(Y_POSITION_MODULE_V)
        self.y_pos_charge_h = await self.convert(Y_POSITION_CHARGER_H)
        self.y_pos_charge_v = await self.convert(Y_POSITION_CHARGER_V)
        self.y_pos_load = await self.convert(Y_POSITION_LOAD)
        self.y_pos_gripper_storage = await self.convert(Y_POSITION_GRIPPER_STORAGE)
        self.y_pos_has_zeroed = await self.convert(Y_POSITION_HAS_ZEROED)
        self.y_pos_powered = await self.convert(Y_POSITION_POWERED)
        self.y_pos_alarm = await self.convert(Y_POSITION_ALARM)

        # Чтение команд с PLC, заготовка
        # await self.handle_plc_commands()

    def check_position(self):
        """Проверка позиции манипулятора по осям"""
        ManipulatorPoints.module_h_available = True if (self.x_pos_module_h and self.y_pos_module_h) else False
        ManipulatorPoints.module_v_available = True if (self.x_pos_module_v and self.y_pos_module_v) else False
        ManipulatorPoints.charge_h_available = True if (self.x_pos_charge_h and self.y_pos_charge_h) else False
        ManipulatorPoints.charge_v_available = True if (self.x_pos_charge_v and self.y_pos_charge_v) else False
        ManipulatorPoints.pos_load_available = True if (self.x_pos_load and self.y_pos_load) else False
        ManipulatorPoints.pos_grippers_available = True if (
                self.x_pos_gripper_storage and self.y_pos_gripper_storage) else False

    async def get_wp_info_and_update_opc_data(self):
        """Узнать текущую позицию и обновить данные в OPC"""
        waypoint_info = await self._get_current_waypoint()
        if waypoint_info.is_valid:
            await self._update_opc_nodes(waypoint_info.name)
        else:
            self.logger.warning("Пропуск обновления OPC узлов из-за ошибки получения точки")


class OPCUAClientVT(OPCUAClient):
    async def read_nodes(self):
        """Чтение всех узлов сервера"""
        self.h_table_hatch_opened = self.convert(H_TABLE_HATCH_OPENED)
        self.h_table_hatch_closed = self.convert(H_TABLE_HATCH_CLOSED)
        self.h_table_hatch_alarm = self.convert(H_TABLE_HATCH_ALARM)
        self.h_table_lift_pos_top = self.convert(H_TABLE_LIFT_POS_TOP)
        self.h_table_lift_pos_bottom = self.convert(H_TABLE_LIFT_POS_BOTTOM)
        self.h_table_lift_alarm = self.convert(H_TABLE_LIFT_ALARM)

        self.h_box_lift_pos_top = self.convert(H_BOX_LIFT_POS_TOP)
        self.h_box_lift_pos_bottom = self.convert(H_BOX_LIFT_POS_BOTTOM)
        self.h_box_lift_alarm = self.convert(H_BOX_LIFT_ALARM)

    def check_position(self):
        """Проверка позиции манипулятора по осям"""
        pass

    def handle_plc_commands(self):
        """Чтение командных узлов с PLC и пересылка в cmd_queue"""
        pass

    async def get_wp_info_and_update_opc_data(self):
        """Узнать текущую позицию и обновить данные в OPC"""
        pass


class OPCUAClientVTOL(OPCUAClient):
    async def read_nodes(self):
        """Чтение всех узлов сервера"""
        self.v_table_hatch_opened = self.convert(V_TABLE_HATCH_OPENED)
        self.v_table_hatch_closed = self.convert(V_TABLE_HATCH_CLOSED)
        self.v_table_hatch_alarm = self.convert(V_TABLE_HATCH_ALARM)
        self.v_table_lift_pos_top = self.convert(V_TABLE_LIFT_POS_TOP)
        self.v_table_lift_pos_bottom = self.convert(V_TABLE_LIFT_POS_BOTTOM)
        self.v_table_lift_alarm = self.convert(V_TABLE_LIFT_ALARM)

    def check_position(self):
        """Проверка позиции манипулятора по осям"""
        pass

    def handle_plc_commands(self):
        """Чтение командных узлов с PLC и пересылка в cmd_queue"""
        pass

    async def get_wp_info_and_update_opc_data(self):
        """Узнать текущую позицию и обновить данные в OPC"""
        pass


# for testing
async def main():
    url = PLC_MANIPULATOR_ADDRESS

    async def browse_recursive(node, depth=0):
        name = await node.read_browse_name()
        print("  " * depth + f"{name.Name} ({node.nodeid})")

        for child in await node.get_children():
            await browse_recursive(child, depth + 1)

    async with Client(url=url) as client:
        root = client.get_root_node()
        print(f"Корневой узел: {root}")

        # objects = client.get_objects_node()
        # await browse_recursive(objects)

        # node = client.get_node("ns=4;s=|var|HCFA-PLC.Application.OPC.BladeStoppers.Stopper2.qCmd")
        node = client.get_node(F"ns=4;s={ENABLE_MOVE_TO_MODULE_H}")
        value = await node.read_value()
        print(ENABLE_MOVE_TO_MODULE_H, value)

        await node.set_value(ua.DataValue(ua.Variant(1, ua.VariantType.Int16)))
        print('BladeStoppers.Stopper2.qCmd: ', value)


if __name__ == "__main__":
    asyncio.run(main())
