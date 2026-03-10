from PyQt5 import QtCore, QtWidgets
from typing import Dict

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QLabel, QFrame, QListWidget, QListWidgetItem
from PyQt5.QtGui import QColor
from datetime import datetime

from commands import Command, CmdType, RobotTrajectories, RobotActions
from config import (POINTS_PATH, TRAJ_PATH, RED_COLOR,
                    LOG_STYLESHEET, LOG_COLOR_NEUTRAL, LOG_COLOR_STOP,
                    LOG_COLOR_OPC, LOG_COLOR_ERROR, LOG_COLOR_SUCCESS,
                    CONN_ONLINE_STYLE, CONN_OFFLINE_STYLE,
                    STOP_BTN_STYLE, POWER_OFF_BTN_STYLE,
                    POWER_ON_ACTIVE_STYLE, POWER_OFF_ACTIVE_STYLE, POWER_BTN_INACTIVE_STYLE, LOG_MAX, COMMON_BTN_STYLE,
                    ACTIVATED_BTN_STYLE)
from ui_form import Ui_Form
from utils import atomic_write_json
from trajectory_map_widget import TrajectoryMapWidget
from available_trajectories import available_trajectories as AVAIL_TRAJS
from states_modes_errors import ControllerState, SafetyStatus, MotionMode, LastError, CONTROLLER_STATE_RU, \
    SAFETY_STATUS_RU, MOTION_MODE_RU, LAST_ERROR_RU
from display_names import POINT_NAMES, ACTION_NAMES, traj_display_name


class MainWindow(QMainWindow):
    """
    Класс - основное окно пользователя.
    """

    def __init__(self, robot_controller, cmd_queue, opc_handler,
                 heartbeat=None, watchdogs=None, cmd_log_queue=None):
        super().__init__()
        self.RobotController = robot_controller
        self.cmd_queue = cmd_queue
        self.Waypoints: Dict[str, dict] = {}
        self.Trajectories: Dict[str, dict] = {}
        self.io0_state: bool = False  # 2025_09_29
        self._heartbeat = heartbeat
        self._plc_clients = watchdogs or {}  # {'manipulator': client, 'vt': client, 'vtol': client}
        self._cmd_log_queue = cmd_log_queue  # очередь OPC-событий
        self._last_nearest_wp: str = ""
        self._log_last_cmd: str = ""
        self._log_last_cs: int = -1
        self._log_last_err: int = 0   # для детекции новых ошибок
        self._pending_cmd: str = ""   # последняя GUI-команда (для атрибуции ошибки)

        self.manipulator_command(
            Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
        self.Waypoints = self.RobotController.get_waypoints_snapshot()
        self.Trajectories = self.RobotController.get_trajectories_snapshot()
        self.Actions = self.RobotController.get_actions_snapshot()
        self.opc_handler = opc_handler

        self.ZGTimer = QtCore.QTimer()
        self.ZGTimer.setInterval(100)  # мс
        self.ZGTimer.timeout.connect(self._zg_tick)

        self.PowerCheckTimer = QtCore.QTimer()
        # self.PowerCheckTimer.timeout.connect(self.update_power_button_state)
        self.PowerCheckTimer.start(1000)

        self.InitUI()

    def InitUI(self):
        """Загружаем конфигурацию окна из дизайнера"""
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.ui.ActivateZG.setCheckable(True)
        self.ui.ActivateZG.toggled.connect(self.manipulator_free_drive)
        self.ui.ActivateSJ.clicked.connect(self.start_simple_joystick)
        self.ui.MoveTrajectory.clicked.connect(self.move_by_selected_trajectory)
        self.ui.ExecuteAction.clicked.connect(self.execute_selected_action)
        self.ui.OutputControl.setCheckable(True)
        self.ui.OutputControl.toggled.connect(self.manipulator_gripper_control)
        self.ui.ShiftGripper.setCheckable(True)
        self.ui.ShiftGripper.toggled.connect(self.manipulator_shift_gripper)
        self.ui.SavePoint.clicked.connect(self.save_current_position)
        self.ui.AddPointToTrajectory.clicked.connect(self.add_current_point_to_trajectory)

        self.ui.comboBox.currentTextChanged.connect(self.waypoint_selected)
        self.ui.TrajectoriesComboBox.currentTextChanged.connect(self.trajectory_selected)
        self.ui.trajListView.clicked.connect(self.on_traj_list_clicked)
        self.ui.actionsListView.clicked.connect(self.on_actions_list_clicked)
        self.ui.StopMove.setStyleSheet(STOP_BTN_STYLE)
        self.ui.StopMove.setStyleSheet(POWER_OFF_BTN_STYLE)
        self.ui.PowerOn.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.ActivateZG.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.ActivateSJ.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.SavePoint.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.MoveToPoint.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.AddPointToTrajectory.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.MoveTrajectory.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.ExecuteAction.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.OutputControl.setStyleSheet(COMMON_BTN_STYLE)
        self.ui.ShiftGripper.setStyleSheet(COMMON_BTN_STYLE)
        self.update_combo_box()
        self.update_trajectories()
        self.update_actions()
        self._init_trajectory_map()
        self.ui.PowerOn.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 1}, source="GUI")
        ))
        self.ui.PowerOff.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 0}, source="GUI")
        ))
        self.ui.MoveToPoint.clicked.connect(self.move_to_selected_point)
        self.ui.StopMove.clicked.connect(self.stop_drive)
        # self.ui.webView.load(QtCore.QUrl("http://192.168.88.100:8080/"))

        # ── Прямое логирование нажатий кнопок ────────────────
        self.ui.PowerOn.clicked.connect(
            lambda: self._add_log_entry("Питание ВКЛ", "→", LOG_COLOR_NEUTRAL))
        self.ui.PowerOff.clicked.connect(
            lambda: self._add_log_entry("Питание ВЫКЛ", "→", LOG_COLOR_NEUTRAL))
        self.ui.StopMove.clicked.connect(
            lambda: self._add_log_entry("Стоп", "→", LOG_COLOR_STOP))
        self.ui.MoveToPoint.clicked.connect(
            lambda: self._add_log_entry(
                f"Перемещение: {self.ui.comboBox.currentData(Qt.UserRole) or self.ui.comboBox.currentText()}",
                "→", LOG_COLOR_NEUTRAL))
        self.ui.MoveTrajectory.clicked.connect(
            lambda: self._add_log_entry(
                f"Траектория: {self.ui.TrajectoriesComboBox.currentData(Qt.UserRole) or self.ui.TrajectoriesComboBox.currentText()}",
                "→", LOG_COLOR_NEUTRAL))
        self.ui.ExecuteAction.clicked.connect(
            lambda: self._add_log_entry("Действие: запрос", "→", LOG_COLOR_NEUTRAL))
        self.ui.ActivateZG.toggled.connect(
            lambda on: self._add_log_entry(
                f"Свободное движение: {'ВКЛ' if on else 'ВЫКЛ'}", "→", LOG_COLOR_NEUTRAL))
        self.ui.ActivateSJ.toggled.connect(
            lambda on: self._add_log_entry(
                f"Джойстик: {'ВКЛ' if on else 'ВЫКЛ'}", "→", LOG_COLOR_NEUTRAL))
        self.ui.OutputControl.toggled.connect(
            lambda on: self._add_log_entry(
                f"Захват: {'ВКЛ' if on else 'ВЫКЛ'}", "→", LOG_COLOR_NEUTRAL))
        self.ui.ShiftGripper.toggled.connect(
            lambda on: self._add_log_entry(
                f"Смещение захвата: {'ВКЛ' if on else 'ВЫКЛ'}", "→", LOG_COLOR_NEUTRAL))

        self._init_op_log_tab()
        self._init_status_bar()
        self._init_stop_toolbar()
        self.show()

    def _init_stop_toolbar(self) -> None:
        """Постоянная панель с кнопкой СТОП, видимой на всех вкладках."""
        toolbar = self.addToolBar("Аварийная остановка")
        toolbar.setMovable(False)
        toolbar.setFloatable(False)

        stop_btn = QtWidgets.QPushButton("■  СТОП")
        font = stop_btn.font()
        font.setPointSize(14)
        font.setBold(True)
        stop_btn.setFont(font)
        stop_btn.setMinimumHeight(48)
        stop_btn.setMinimumWidth(160)
        stop_btn.setStyleSheet(STOP_BTN_STYLE)
        stop_btn.clicked.connect(self.stop_drive)
        toolbar.addWidget(stop_btn)

        power_off_btn = QtWidgets.QPushButton("⏻  Питание ВЫКЛ")
        power_off_btn.setFont(font)
        power_off_btn.setMinimumHeight(48)
        power_off_btn.setMinimumWidth(180)
        power_off_btn.setStyleSheet(POWER_OFF_BTN_STYLE)
        power_off_btn.clicked.connect(lambda: self.cmd_queue.put(
            Command(CmdType.POWER, {'state': 0}, source="GUI")))
        power_off_btn.clicked.connect(
            lambda: self._add_log_entry("Питание ВЫКЛ", "→", LOG_COLOR_NEUTRAL))
        toolbar.addWidget(power_off_btn)

    def _init_status_bar(self) -> None:
        """Создаёт постоянную панель статуса робота в нижней строке окна."""
        sb = self.statusBar()
        sb.setSizeGripEnabled(False)

        # --- Индикаторы подключения (левая сторона) ---
        self._conn_labels: Dict[str, QLabel] = {}
        conn_items = [
            ('rc',          'RC'),
            ('opc_server',  'OPC Srv'),
            ('manipulator', 'ПЛК M'),
            ('vt',          'ПЛК VT'),
            ('vtol',        'ПЛК VTOL'),
        ]
        for key, display in conn_items:
            lbl = QLabel(f"● {display}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFrameStyle(QFrame.Panel | QFrame.Sunken)
            lbl.setStyleSheet("padding:2px 6px; color: #9e9e9e;")
            sb.addWidget(lbl)
            self._conn_labels[key] = lbl

        # --- Статус робота (правая сторона) ---
        self._lbl_state = QLabel("Состояние: —")
        self._lbl_safety = QLabel("Безопасность: —")
        self._lbl_mode = QLabel("Режим: —")
        self._lbl_error = QLabel("Ошибка: —")

        for lbl in (self._lbl_state, self._lbl_safety, self._lbl_mode, self._lbl_error):
            lbl.setMinimumWidth(180)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFrameStyle(QFrame.Panel | QFrame.Sunken)
            lbl.setStyleSheet("padding:2px 8px;")
            sb.addPermanentWidget(lbl)

        self.StatusTimer = QtCore.QTimer()
        self.StatusTimer.timeout.connect(self._update_status)
        self.StatusTimer.start(500)

    def _update_conn_indicators(self) -> None:
        """Обновляет индикаторы подключения RC, OPC Server и трёх ПЛК."""
        # RC и OPC Server — через heartbeat
        if self._heartbeat is not None:
            hb = self._heartbeat.state()
            for key in ('rc', 'opc_server'):
                lbl = self._conn_labels.get(key)
                if lbl is None:
                    continue
                alive = hb.get(key, False)
                lbl.setStyleSheet(
                    CONN_ONLINE_STYLE if alive else CONN_OFFLINE_STYLE
                )

        # Три ПЛК-клиента — через client.is_connected
        for key in ('manipulator', 'vt', 'vtol'):
            lbl = self._conn_labels.get(key)
            client = self._plc_clients.get(key)
            if lbl is None or client is None:
                continue
            alive = getattr(client, 'is_connected', False)
            lbl.setStyleSheet(
                CONN_ONLINE_STYLE if alive else CONN_OFFLINE_STYLE
            )

    def _update_status(self) -> None:
        """Обновляет метки статуса на основе текущего состояния робота."""
        self._update_conn_indicators()
        try:
            state = self.RobotController.get_state_snapshot()

            cs = state.controller_state
            if not isinstance(cs, ControllerState):
                cs = ControllerState(cs)
            text, style = CONTROLLER_STATE_RU.get(cs, (cs.name, ""))
            self._lbl_state.setText(f"Состояние: {text}")
            self._lbl_state.setStyleSheet(f"padding:2px 8px;{style}")

            ss = state.safety_status
            if not isinstance(ss, SafetyStatus):
                ss = SafetyStatus(ss)
            text, style = SAFETY_STATUS_RU.get(ss, (ss.name, ""))
            self._lbl_safety.setText(f"Безопасность: {text}")
            self._lbl_safety.setStyleSheet(f"padding:2px 8px;{style}")

            mode = state.mode
            if not isinstance(mode, MotionMode):
                try:
                    mode = MotionMode(mode)
                except (ValueError, TypeError):
                    mode = None
            if mode is not None:
                text, style = MOTION_MODE_RU.get(mode, (str(mode), ""))
            else:
                text, style = "—", ""
            self._lbl_mode.setText(f"Режим: {text}")
            self._lbl_mode.setStyleSheet(f"padding:2px 8px;{style}")

            err = state.last_error
            if not isinstance(err, LastError):
                try:
                    err = LastError(err)
                except (ValueError, TypeError):
                    err = None
            if err is not None:
                text, style = LAST_ERROR_RU.get(err, (str(err), "background:#ffcdd2"))
            else:
                text, style = "—", ""
            self._lbl_error.setText(f"Ошибка: {text}")
            self._lbl_error.setStyleSheet(f"padding:2px 8px;{style}")

            nearest_info = self.RobotController.get_nearest_info()
            nearest_wp = (nearest_info or {}).get('waypoint') or ""
            if nearest_wp and nearest_wp != self._last_nearest_wp:
                self._last_nearest_wp = nearest_wp
                self.trajectory_map.set_current_position(nearest_wp)

        except Exception:
            pass

        try:
            state = self.RobotController.get_state_snapshot()
            self._update_op_log(
                last_cmd=getattr(state, 'last_command', None) or "",
                cmd_state=getattr(state, 'cmd_state', 0) or 0,
            )
        except Exception:
            pass

        try:
            state = self.RobotController.get_state_snapshot()
            raw_err = state.last_error
            if raw_err is not None:
                err = raw_err if isinstance(raw_err, LastError) else LastError(raw_err)
                err_val = err.value
                if err_val != 0 and err_val != self._log_last_err:
                    err_text, _ = LAST_ERROR_RU.get(err, (str(err), ""))
                    context = f"{self._pending_cmd} — {err_text}" if self._pending_cmd else err_text
                    self._add_log_entry(context, "✗", LOG_COLOR_ERROR)
                self._log_last_err = err_val
        except Exception:
            pass

        # OPC-команды из очереди (синий цвет, префикс [OPC])
        try:
            if self._cmd_log_queue is not None:
                while not self._cmd_log_queue.empty():
                    msg = self._cmd_log_queue.get_nowait()
                    self._add_log_entry(f"[OPC]  {msg}", "→", LOG_COLOR_OPC)
        except Exception:
            pass

    # ── Журнал операций ───────────────────────────────────────
    def _init_op_log_tab(self) -> None:
        """Добавляет 4 вкладку 'Журнал' с QListWidget последних операций."""
        tab = QtWidgets.QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        hdr = QLabel("Журнал операций (последние 100)")
        hdr.setFont(QFont("Segoe UI", 10, QFont.Bold))
        layout.addWidget(hdr)

        self._op_log = QListWidget()
        self._op_log.setSpacing(1)
        self._op_log.setStyleSheet(LOG_STYLESHEET)
        layout.addWidget(self._op_log)

        self.ui.tabWidget.addTab(tab, "Журнал")

    def _add_log_entry(self, command: str, icon: str, fg: str) -> None:
        """Добавляет строку в журнал, удаляет лишние при переполнении."""
        if icon == "→" and not command.startswith("[OPC]"):
            self._pending_cmd = command  # запоминаем для атрибуции возможной ошибки
            self._log_last_err = 0       # сбрасываем ошибку, чтобы та же ошибка снова отобразилась
        ts = datetime.now().strftime("%H:%M:%S")
        text = f"{ts}  {icon}  {command}"
        item = QListWidgetItem(text)
        item.setForeground(QColor(fg))
        self._op_log.insertItem(0, item)
        while self._op_log.count() > LOG_MAX:
            self._op_log.takeItem(self._op_log.count() - 1)

    def _update_op_log(self, last_cmd: str, cmd_state: int) -> None:
        """Добавляет запись о результате команды при изменении last_command или cmd_state."""
        cmd_changed = last_cmd != self._log_last_cmd
        cs_changed = cmd_state != self._log_last_cs
        if not cmd_changed and not cs_changed:
            return

        prev_cs = self._log_last_cs
        prev_cmd = self._log_last_cmd
        self._log_last_cs = cmd_state
        self._log_last_cmd = last_cmd

        # Для читаемости метки в журнале: предпочитаем текст кнопки (_pending_cmd),
        # fallback на внутреннее имя last_command
        label = self._pending_cmd or last_cmd or prev_cmd

        # Траекторные команды: смотрим на переход cmd_state в терминальную зону
        if prev_cs < 200 and 200 <= cmd_state < 300:
            self._add_log_entry(label, "✓", LOG_COLOR_SUCCESS)
            QtWidgets.QMessageBox.information(self, "Выполнено", f"✓  {label}")
            self._pending_cmd = ""
        elif prev_cs < 300 and 300 <= cmd_state < 400:
            self._add_log_entry(label, f"✗  ошибка (код {cmd_state})", LOG_COLOR_ERROR)
            self._pending_cmd = ""
        elif prev_cs < 400 and cmd_state >= 400:
            self._add_log_entry(label, "✗  заблокировано", LOG_COLOR_ERROR)
            self._pending_cmd = ""
        elif cmd_changed and last_cmd and cmd_state < 100:
            # Не-траекторная команда (PowerOn/Off, MoveToPoint…):
            # last_command обновляется ПОСЛЕ успешного выполнения → это подтверждение "✓"
            self._add_log_entry(self._pending_cmd or last_cmd, "✓", LOG_COLOR_SUCCESS)
            self._pending_cmd = ""

    def _init_trajectory_map(self) -> None:
        """Встраивает TrajectoryMapWidget в плейсхолдер 3 вкладки."""
        placeholder = self.ui.trajectoryMapPlaceholder
        layout = QVBoxLayout(placeholder)
        layout.setContentsMargins(0, 0, 0, 0)

        self.trajectory_map = TrajectoryMapWidget(placeholder)
        layout.addWidget(self.trajectory_map)

        # Клик по узлу → выбрать точку в комбобоксе Tab 1
        self.trajectory_map.node_clicked.connect(self._on_map_node_clicked)

        # Обновляем карту при смене вкладки
        self.ui.tabWidget.currentChanged.connect(self._on_tab_changed)

    def _on_map_node_clicked(self, point_name: str) -> None:
        """
        Клик по узлу карты:
        - выбирает точку в комбобоксе Tab 1
        - если текущая позиция известна, ищет прямую траекторию и подсвечивает ребро
        - синхронизирует выбор траектории с TrajectoriesComboBox Tab 1
        - Оператор видит подсветку на карте
        """
        # Синхронизация точки с Tab 1 (поиск по внутреннему имени в UserRole)
        model = self.ui.comboBox.model()
        index = next(
            (i for i in range(model.rowCount())
             if model.item(i).data(Qt.UserRole) == point_name),
            -1
        )
        if index >= 0:
            self.ui.comboBox.setCurrentIndex(index)

        src = self.trajectory_map._current_point.get('waypoint')
        if not src or src == point_name:
            return

        traj_name = self._find_direct_trajectory(src, point_name)
        if traj_name:
            # Подсветка ребра на карте
            self.trajectory_map.highlight_trajectory(src, point_name, traj_name)
            # Синхронизация с TrajectoriesComboBox и TrajectoryName на Tab 1
            tmodel = self.ui.TrajectoriesComboBox.model()
            tidx = next(
                (i for i in range(tmodel.rowCount())
                 if tmodel.item(i).data(Qt.UserRole) == traj_name),
                -1
            )
            if tidx >= 0:
                self.ui.TrajectoriesComboBox.setCurrentIndex(tidx)
            self.ui.TrajectoryName.setText(traj_name)
        else:
            self.trajectory_map._info_label.setText(
                f"Нет прямой траектории: {src} → {point_name}")
            self.trajectory_map._info_label.setStyleSheet(
                RED_COLOR)

    def _find_direct_trajectory(self, src: str, dst: str) -> str | None:
        """
        Ищет прямую траекторию между src и dst через available_trajectories.
        Проверяет оба направления: src→dst и dst→src.
        """
        dst_short = dst[1:] if dst.startswith('p') else dst
        src_short = src[1:] if src.startswith('p') else src

        # Прямое направление: из src в dst
        for traj in AVAIL_TRAJS.get(src, set()):
            if f"_To_{dst_short}" in traj and traj in self.Trajectories:
                return traj

        # Обратное направление: из dst в src
        for traj in AVAIL_TRAJS.get(dst, set()):
            if f"_To_{src_short}" in traj and traj in self.Trajectories:
                return traj

        return None

    def _on_tab_changed(self, index: int) -> None:
        """При переходе на вкладку карты — обновляем текущую позицию."""
        if index == 2:
            try:
                nearest = self.RobotController.find_nearest_waypoint()
                if nearest:
                    self.trajectory_map.set_current_position(nearest)
            except Exception:
                pass
        else:
            # Уходим с карты — сбрасываем подсветку рёбер
            if hasattr(self, 'trajectory_map'):
                self.trajectory_map.reset_highlight()

    def update_power_button_state(self) -> None:
        is_running = (self.RobotController.get_controller_state() == 'run')
        if is_running:
            self.ui.PowerOn.setStyleSheet(POWER_ON_ACTIVE_STYLE)
            self.ui.PowerOff.setStyleSheet(POWER_BTN_INACTIVE_STYLE)
        else:
            self.ui.PowerOn.setStyleSheet(POWER_BTN_INACTIVE_STYLE)
            self.ui.PowerOff.setStyleSheet(POWER_OFF_ACTIVE_STYLE)

    def save_waypoints(self) -> bool:
        try:
            waypoints_list = list(self.Waypoints.values())
            atomic_write_json(POINTS_PATH, {"waypoints": waypoints_list})
            return True
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save waypoints: {e}")
            return False

    def save_trajectories(self) -> bool:
        try:
            trajectories_list = list(self.Trajectories.values())
            atomic_write_json(TRAJ_PATH, {"trajectories": trajectories_list})
            return True
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save trajectories: {e}")
            return False

    def update_combo_box(self) -> None:
        items_model = QStandardItemModel()
        for point in self.Waypoints.keys():
            display = POINT_NAMES.get(point, point)
            item = QStandardItem(display)
            item.setData(point, Qt.UserRole)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.comboBox.setModel(items_model)

    def update_trajectories(self) -> None:
        items_model = QStandardItemModel()
        for traj in self.Trajectories.keys():
            display = traj_display_name(traj)
            item = QStandardItem(display)
            item.setData(traj, Qt.UserRole)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.TrajectoriesComboBox.setModel(items_model)
        self.ui.trajListView.setModel(items_model)

    def update_actions(self) -> None:
        items_model = QStandardItemModel()
        for action in self.Actions.keys():
            display = ACTION_NAMES.get(action, action)
            item = QStandardItem(display)
            item.setData(action, Qt.UserRole)
            item.setEditable(False)
            items_model.appendRow(item)
        self.ui.actionsListView.setModel(items_model)

    def on_traj_list_clicked(self) -> None:
        index = self.ui.trajListView.currentIndex()
        internal = index.data(Qt.UserRole) or index.data()
        self.ui.TrajectoryName.setText(internal)

    def on_actions_list_clicked(self) -> None:
        index = self.ui.actionsListView.currentIndex()
        internal = index.data(Qt.UserRole) or index.data()
        self.ui.ActionName.setText(internal)

    def trajectory_selected(self, _display_name) -> None:
        internal = self.ui.TrajectoriesComboBox.currentData(Qt.UserRole) or _display_name
        self.ui.TrajectoryName.setText(internal)

    def waypoint_selected(self, _display_name) -> None:
        point_name = self.ui.comboBox.currentData(Qt.UserRole) or _display_name
        self.ui.PointName.setText(point_name)
        if point_name in self.Waypoints:
            wp = self.Waypoints[point_name]
            self.ui.SetSpeed.setText(str(wp.get('speed', 0.5)))
            self.ui.SetAccel.setText(str(wp.get('accel', 0.5)))
            self.ui.SetBlend.setText(str(wp.get('blend', 0.0)))

    # def trajectory_selected(self, trajectory_name):
    #     print(trajectory_name)

    def manipulator_command(self, cmd: Command) -> None:
        self.cmd_queue.put(cmd)

    def manipulator_gripper_control(self, clamp: bool) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.IO_SET, {'index': 0, 'value': clamp}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "I/O error",
                                           f"Не удалось изменить состояние DO0: {e}")
            self.ui.OutputControl.setChecked(False)

    def manipulator_shift_gripper(self, shift: bool) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.IO_SET, {'index': 1, 'value': shift}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "I/O error",
                                           f"Не удалось изменить состояние DO1: {e}")
            # self.ui.ShiftGripper.setChecked(False)

    def manipulator_free_drive(self, activate: bool) -> None:  # 2025_09_29
        try:
            self._log_last_err = 0
            if activate:
                self.ui.ActivateZG.setStyleSheet(ACTIVATED_BTN_STYLE)
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
                if not self.ZGTimer.isActive():
                    self.ZGTimer.start()
            else:
                self.ui.ActivateZG.setStyleSheet(COMMON_BTN_STYLE)
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            if self.ZGTimer.isActive():
                self.ZGTimer.stop()

    def manipulator_free_drive_old(self, activate: bool) -> None:
        try:
            self.manipulator_command(
                Command(CmdType.FREE_DRIVE, {'state': 1 if activate else 2},
                        source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)

    def add_current_point_to_trajectory(self) -> None:
        point_name = self.ui.PointName.text().strip()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select existing point!")
            return

        trajectory_name = self.ui.TrajectoryName.text().strip()
        if not trajectory_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select trajectory!")
            return

        if point_name in self.Waypoints and trajectory_name in self.Trajectories:
            positions = self.Trajectories[trajectory_name].get('positions')
            positions.append({'name': point_name, 'motion': 'joint'})

            new_trajectory = {
                "name": trajectory_name,
                "positions": positions
            }

            self.Trajectories[trajectory_name] = new_trajectory
            if self.save_trajectories():
                QtWidgets.QMessageBox.information(
                    None,
                    "Success",
                    f"Point '{point_name}' successfully added to trajectory {trajectory_name}!")

    def save_current_position(self) -> None:
        point_name = self.ui.PointName.text().strip()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please enter point name!")
            return

        try:
            tcp_position = self.RobotController.get_current_tcp_position()
            joint_position = self.RobotController.get_current_joint_position()
            speed = float(self.ui.SetSpeed.text())
            accel = float(self.ui.SetAccel.text())
            blend = float(self.ui.SetBlend.text())

            new_point = {
                "name": point_name,
                "speed": speed,
                "accel": accel,
                "blend": blend,
                "tcp": {
                    "x": tcp_position[0],
                    "y": tcp_position[1],
                    "z": tcp_position[2],
                    "Rx": tcp_position[3],
                    "Ry": tcp_position[4],
                    "Rz": tcp_position[5]
                },
                "joints": {
                    "J1": joint_position[0],
                    "J2": joint_position[1],
                    "J3": joint_position[2],
                    "J4": joint_position[3],
                    "J5": joint_position[4],
                    "J6": joint_position[5]
                }
            }

            self.Waypoints[point_name] = new_point

            if self.save_waypoints():
                self.update_combo_box()
                QtWidgets.QMessageBox.information(None, "Success",
                                                  f"Point '{point_name}' saved successfully!")
        except ValueError:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please enter valid parameters!")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to save point: {e}")

    def stop_drive(self) -> None:
        self.manipulator_command(
            Command(CmdType.STOP_MOVE, {}, source="GUI"))

    def move_to_selected_point(self) -> None:
        point_name = self.ui.comboBox.currentData(Qt.UserRole) or self.ui.comboBox.currentText()
        if not point_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a point first!")
            return
        try:
            self.manipulator_command(
                Command(CmdType.REFRESH_WAYPOINTS, {}, source="GUI"))
            motion = "line" if self.ui.chkLineMotion.isChecked() else "joint"
            self.manipulator_command(
                Command(CmdType.MOVE_TO_POINT,
                        {'name': point_name, 'motion': motion},
                        source="GUI"))

            # QtWidgets.QMessageBox.information(None, "Success",
            #                                   f"Moving to point '{point_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move to point: {e}")

    def move_by_selected_trajectory(self) -> None:
        trajectory_name = self.ui.TrajectoryName.text()
        if not trajectory_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select a trajectory first!")
            return

        try:
            cmd_enum = getattr(RobotTrajectories, trajectory_name)
            # команда через OPC
            self.opc_handler.set_trajectory(cmd_enum.value)

            # команда напрямую в манипулятор
            # self.manipulator_command(
            #     Command(CmdType.EXECUTE_TRAJECTORY, {'num': cmd_enum}, source="GUI"))

            # QtWidgets.QMessageBox.information(None, "Success",
            #                                   f"Moving by trajectory '{trajectory_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to move by trajectory: {e}")

    # def execute_selected_route(self) -> None:
    #     route_name = self.ui.ActionName.text()
    #     if not route_name:
    #         QtWidgets.QMessageBox.warning(None, "Warning",
    #                                       "Please select a route first!")
    #         return
    #     try:
    #         # команда через OPC
    #         route_enum = getattr(RobotRoutes, route_name)
    #         self.opc_handler.set_route(route_enum.value)
    #
    #         QtWidgets.QMessageBox.information(None, "Success",
    #                                           f"Executing route '{route_name}'")
    #     except Exception as e:
    #         QtWidgets.QMessageBox.critical(None, "Error",
    #                                        f"Failed to execute route: {e}")

    def execute_selected_action(self) -> None:
        action_name = self.ui.ActionName.text()
        if not action_name:
            QtWidgets.QMessageBox.warning(None, "Warning",
                                          "Please select an action first!")
            return
        try:
            # команда через OPC
            action_enum = getattr(RobotActions, action_name)
            self.opc_handler.set_action(action_enum.value)

            # QtWidgets.QMessageBox.information(None, "Success",
            #                                   f"Executing action '{action_name}'")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to execute action: {e}")

    def _zg_tick(self) -> None:  # 2025_09_29
        try:
            if self.ui.ActivateZG.isChecked():
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 1}, source="GUI")
                )
            else:
                self.manipulator_command(
                    Command(CmdType.FREE_DRIVE, {'state': 2}, source="GUI")
                )
                self.ZGTimer.stop()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Zero Gravity periodic toggle failed: {e}")
            self.ui.ActivateZG.setChecked(False)
            self.ZGTimer.stop()

    def start_simple_joystick(self) -> None:
        try:
            self._log_last_err = 0
            self.manipulator_command(
                Command(CmdType.START_SIMPLE_JOYSTICK,
                        {'coord_sys': None}, source="GUI"))
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error",
                                           f"Failed to start simple joystick: {e}")
