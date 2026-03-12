"""
Интерактивная карта траекторий манипулятора ЭРИ Порта.
Визуализация зон, базовых точек и маршрутов перемещения.
Отображает состояния XY-платформы и оборудования из ПЛК.
"""
from PyQt5.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsTextItem,
    QGraphicsPathItem, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGraphicsItem, QFrame,
)
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QLineF
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QFont, QPainterPath, QPainter,
)
import math

from config import (
    GREEN_COLOR, BEIGE_COLOR, BLUE_COLOR, ALABASTER_COLOR,
    ZONE_COLORS, ZONE_BORDER_COLORS,
    NODE_BASE_COLOR, NODE_ENDPOINT_COLOR, NODE_HOME_COLOR,
    NODE_CURRENT_COLOR, NODE_HOVER_COLOR, NODE_BLOCKED_COLOR,
    PEN_NORMAL, PEN_DIM, PEN_HL, PEN_BACK_ARROW,
    MAP_STATUS_OK, MAP_STATUS_WARN, MAP_STATUS_ALM, MAP_STATUS_OFF, ZONE_BLOCK_MAP, GROUPS, SEP_COLOR,
)


# ─── StatusChipItem ───────────────────────────────────────────
class StatusChipItem(QGraphicsRectItem):
    """
    Маленький цветной чип с текстовой подписью.
    Используется для отображения состояния оборудования/платформы на карте.
    """
    _COLORS = {
        'green': (QColor(200, 230, 201), QColor(46, 125, 50)),
        'yellow': (QColor(255, 249, 196), QColor(230, 81, 0)),
        'red': (QColor(255, 205, 210), QColor(183, 28, 28)),
        'gray': (QColor(224, 224, 224), QColor(97, 97, 97)),
    }

    def __init__(self, text: str, x: float, y: float,
                 color_mode: str = 'gray', w: int = 56, h: int = 14):
        super().__init__(0, 0, w, h)
        self.setPos(x, y)
        self.setZValue(20)
        self._txt = QGraphicsTextItem(text, self)
        self._txt.setFont(QFont("Segoe UI", 6, QFont.Bold))
        self._txt.setZValue(21)
        self._txt.setPos(2, -1)
        self.set_color(color_mode)

    def set_color(self, color_mode: str):
        bg_color, fg_color = self._COLORS.get(color_mode, self._COLORS['gray'])
        self.setBrush(QBrush(bg_color))
        self.setPen(QPen(fg_color.darker(110), 1))
        self._txt.setDefaultTextColor(fg_color)

    def update_state(self, text: str, color_mode: str):
        self._txt.setPlainText(text)
        self.set_color(color_mode)


# ─── NodeItem ────────────────────────────────────────────────
class NodeItem(QGraphicsEllipseItem):
    """Интерактивная точка (узел) на карте траекторий."""

    def __init__(self, point_name, display_name, x, y, radius=18,
                 color=NODE_BASE_COLOR, is_endpoint=False, parent_widget=None):
        super().__init__(-radius, -radius, radius * 2, radius * 2)
        self.point_name = point_name
        self.display_name = display_name
        self.radius = radius
        self.base_color = color
        self.is_endpoint = is_endpoint
        self.parent_widget = parent_widget
        self._is_blocked = False

        self.setPos(x, y)
        self.setZValue(10)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        self.setBrush(QBrush(color))
        self.setPen(QPen(color.darker(140), 2))

        # Метка внутри круга
        self._label = QGraphicsTextItem(display_name, self)
        self._label.setFont(QFont("Segoe UI", 7, QFont.Bold))
        self._label.setDefaultTextColor(Qt.white)
        br = self._label.boundingRect()
        self._label.setPos(-br.width() / 2, -br.height() / 2)

        # Тултип
        self._tooltip = QGraphicsTextItem(point_name, self)
        self._tooltip.setFont(QFont("Segoe UI", 8))
        self._tooltip.setDefaultTextColor(QColor(30, 30, 30))
        self._tooltip.setZValue(100)
        self._tooltip.setVisible(False)
        tbr = self._tooltip.boundingRect()
        self._tooltip.setPos(-tbr.width() / 2, -radius - tbr.height() - 4)

    def hoverEnterEvent(self, event):
        if not self._is_blocked:
            self.setBrush(QBrush(NODE_HOVER_COLOR))
            self.setPen(QPen(NODE_HOVER_COLOR.darker(150), 3))
        self._tooltip.setVisible(True)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self.setBrush(QBrush(self.base_color))
        self.setPen(QPen(self.base_color.darker(140) if not self._is_blocked
                         else QColor(183, 28, 28), 2,
                         Qt.DashLine if self._is_blocked else Qt.SolidLine))
        self._tooltip.setVisible(False)
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.parent_widget:
            self.parent_widget.node_clicked.emit(self.point_name)
        super().mousePressEvent(event)

    def set_current(self, is_current: bool):
        if is_current:
            self.base_color = NODE_CURRENT_COLOR
            self.setBrush(QBrush(NODE_CURRENT_COLOR))
            self.setPen(QPen(NODE_CURRENT_COLOR.darker(140), 3))
            self.setScale(1.3)
        else:
            if self._is_blocked:
                # Восстанавливаем заблокированный вид
                self._apply_blocked_style()
            else:
                color = (NODE_HOME_COLOR if self.point_name == "pHomePosition"
                         else NODE_ENDPOINT_COLOR if self.is_endpoint
                else NODE_BASE_COLOR)
                self.base_color = color
                self.setBrush(QBrush(color))
                self.setPen(QPen(color.darker(140), 2))
                self.setScale(1.0)

    def set_blocked(self, blocked: bool, reason: str = ""):
        self._is_blocked = blocked
        if blocked:
            self._apply_blocked_style()
            self._tooltip.setPlainText(f"{self.point_name}\n🔒 {reason}")
        else:
            color = (NODE_HOME_COLOR if self.point_name == "pHomePosition"
                     else NODE_ENDPOINT_COLOR if self.is_endpoint
            else NODE_BASE_COLOR)
            self.base_color = color
            self.setBrush(QBrush(color))
            self.setPen(QPen(color.darker(140), 2))
            self.setScale(1.0)
            self._tooltip.setPlainText(self.point_name)

    def _apply_blocked_style(self):
        self.base_color = NODE_BLOCKED_COLOR
        self.setBrush(QBrush(NODE_BLOCKED_COLOR))
        self.setPen(QPen(QColor(183, 28, 28), 2, Qt.DashLine))
        self.setScale(1.0)


# ─── Стрелка ─────────────────────────────────────────────────
def _make_arrow_path(p1: QPointF, p2: QPointF, arrow_size=8) -> QPainterPath:
    line = QLineF(p1, p2)
    angle = math.atan2(-line.dy(), line.dx())
    ap1 = p2 - QPointF(math.cos(angle - math.pi / 6) * arrow_size,
                       -math.sin(angle - math.pi / 6) * arrow_size)
    ap2 = p2 - QPointF(math.cos(angle + math.pi / 6) * arrow_size,
                       -math.sin(angle + math.pi / 6) * arrow_size)
    path = QPainterPath()
    path.moveTo(p1)
    path.lineTo(p2)
    path.moveTo(p2)
    path.lineTo(ap1)
    path.moveTo(p2)
    path.lineTo(ap2)
    return path


# ─── Главный виджет ──────────────────────────────────────────
class TrajectoryMapWidget(QWidget):
    """
    Виджет с интерактивной картой траекторий манипулятора.
    Показывает текущую позицию манипулятора, состояния XY-платформы
    и оборудования из ПЛК (VT/VTOL столы).
    """
    node_clicked = pyqtSignal(str)

    NODE_LAYOUT = {
        "pHomePosition": (500, 400, "Home", False),

        "pHelicopterModule": (200, 400, "pH", False),
        "pHelicopter1": (120, 250, "pH1", False),
        "pHelicopter1Load": (120, 150, "pH1L", True),
        "pHelicopter2": (120, 550, "pH2", False),
        "pHelicopter2Load": (120, 650, "pH2L", True),

        "pPayload": (370, 250, "pL", False),
        "pPayload1": (310, 150, "pL1", True),
        "pPayload2": (430, 150, "pL2", True),

        "pGrippers": (560, 250, "pG", False),
        "pGrippers1": (500, 150, "pG1", True),
        "pGrippers2": (620, 150, "pG2", True),

        "pCharger": (420, 550, "pC", False),
        "pCharger1": (370, 650, "pC1", True),
        "pCharger2": (470, 650, "pC2", True),

        "pVTOLModule": (800, 400, "pV", False),
        "pVTOL1": (880, 250, "pV1", False),
        "pVTOL1Battery": (830, 150, "pV1B", True),
        "pVTOL1Load": (930, 150, "pV1L", True),
        "pVTOL2": (880, 550, "pV2", False),
        "pVTOL2Battery": (830, 650, "pV2C", True),
        "pVTOL2Battery2": (930, 650, "pV2C2", False),
        "pVTOL2Battery2Charge": (980, 750, "pV2C2С", True),
    }

    EDGES = [
        ("pHomePosition", "pHelicopterModule"),
        ("pHomePosition", "pPayload"),
        ("pHomePosition", "pGrippers"),
        ("pHomePosition", "pCharger"),
        ("pHomePosition", "pVTOLModule"),

        ("pHelicopterModule", "pHelicopter1"),
        ("pHelicopterModule", "pHelicopter2"),
        ("pHelicopter1", "pHelicopter1Load"),
        ("pHelicopter2", "pHelicopter2Load"),

        ("pPayload", "pPayload1"),
        ("pPayload", "pPayload2"),

        ("pGrippers", "pGrippers1"),
        ("pGrippers", "pGrippers2"),

        ("pCharger", "pCharger1"),
        ("pCharger", "pCharger2"),

        ("pVTOLModule", "pVTOL1"),
        ("pVTOLModule", "pVTOL2"),
        ("pVTOL1", "pVTOL1Battery"),
        ("pVTOL1", "pVTOL1Load"),
        ("pVTOL2", "pVTOL2Battery"),
        ("pVTOL2", "pVTOL2Battery2"),
        ("pVTOL2Battery2", "pVTOL2Battery2Charge"),
    ]

    ZONES = [
        (20, 100, 210, 600, "helicopter", "Модуль обслуживания\nвертолёта"),
        (280, 100, 180, 200, "service_load", "Зона полезной нагрузки"),
        (470, 100, 180, 200, "service_grip", "Зона захватов"),
        (310, 500, 230, 200, "charger", "Зарядная станция"),
        (760, 100, 200, 610, "vtol", "Модуль обслуживания\nВТОЛ"),
    ]

    DRONE_LABELS = [
        (120, 210, "ВТ-30Е", QColor(41, 98, 255)),
        (120, 510, "Альфа-Е", QColor(41, 98, 255)),
        (880, 210, "InnoVTOL-3e", QColor(76, 175, 80)),
        (880, 510, "Легионер Е29", QColor(76, 175, 80)),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_point = None
        self._nodes: dict[str, NodeItem] = {}
        self._edge_items: dict[tuple, list] = {}
        self._zone_chips: dict[str, StatusChipItem] = {}  # zone_type + "_x"/"_y" → chip
        self._equip_chips: dict[str, StatusChipItem] = {}  # key → equipment chip
        self._status_labels: dict[str, QLabel] = {}  # key → bottom panel label
        self._init_ui()

    # ── UI ────────────────────────────────────────────────────
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        title = QLabel("Схема траекторий манипулятора ЭРИ Порта")
        title.setFont(QFont("Segoe UI", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Легенда
        legend = QHBoxLayout()
        legend.addStretch()
        for lbl_text, node_color in [
            ("Home", NODE_HOME_COLOR),
            ("Базовая точка", NODE_BASE_COLOR),
            ("Конечная точка", NODE_ENDPOINT_COLOR),
            ("Текущая позиция", NODE_CURRENT_COLOR),
            ("Зона недоступна", NODE_BLOCKED_COLOR),
        ]:
            dot = QLabel("●")
            dot.setFont(QFont("Segoe UI", 14))
            dot.setStyleSheet(f"color: {node_color.name()};")
            txt = QLabel(lbl_text)
            txt.setFont(QFont("Segoe UI", 9))
            legend.addWidget(dot)
            legend.addWidget(txt)
            legend.addSpacing(12)
        legend.addStretch()
        layout.addLayout(legend)

        self._info_label = QLabel("Нажмите на точку для выбора маршрута")
        self._info_label.setFont(QFont("Segoe UI", 10))
        self._info_label.setAlignment(Qt.AlignCenter)
        self._info_label.setStyleSheet(BLUE_COLOR)
        layout.addWidget(self._info_label)

        self._scene = QGraphicsScene()
        self._view = QGraphicsView(self._scene)
        self._view.setRenderHint(QPainter.Antialiasing)
        self._view.setRenderHint(QPainter.SmoothPixmapTransform)
        self._view.setDragMode(QGraphicsView.ScrollHandDrag)
        self._view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self._view.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self._view.setStyleSheet(ALABASTER_COLOR)
        layout.addWidget(self._view, stretch=1)

        # ── Панель состояния ПЛК ──────────────────────────────
        layout.addWidget(self._build_status_panel())

        self._build_scene()
        self._view.fitInView(
            self._scene.sceneRect().adjusted(-30, -30, 30, 30),
            Qt.KeepAspectRatio)

        self.node_clicked.connect(self._on_node_clicked)

    def _build_status_panel(self) -> QWidget:
        """Нижняя панель с компактными индикаторами ПЛК."""
        panel = QWidget()
        row = QHBoxLayout(panel)
        row.setContentsMargins(6, 2, 6, 2)
        row.setSpacing(4)

        for group_name, items in GROUPS:
            hdr = QLabel(group_name + ":")
            hdr.setFont(QFont("Segoe UI", 8, QFont.Bold))
            row.addWidget(hdr)
            for key, default_text in items:
                lbl = QLabel(default_text)
                lbl.setFont(QFont("Segoe UI", 8))
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setFrameStyle(QFrame.Panel | QFrame.Sunken)
                lbl.setStyleSheet(MAP_STATUS_OFF)
                lbl.setMinimumWidth(80)
                row.addWidget(lbl)
                self._status_labels[key] = lbl

            sep = QFrame()
            sep.setFrameShape(QFrame.VLine)
            sep.setStyleSheet(SEP_COLOR)
            row.addWidget(sep)

        row.addStretch()
        return panel

    # ── Сцена ─────────────────────────────────────────────────
    def _build_scene(self):
        self._scene.clear()
        self._nodes.clear()
        self._edge_items.clear()
        self._zone_chips.clear()
        self._equip_chips.clear()

        # 1) Зоны + XY-чип платформы в каждой зоне
        for (zone_x, zone_y, zone_w, zone_h, zone_type, zone_label) in self.ZONES:
            rect = QGraphicsRectItem(zone_x, zone_y, zone_w, zone_h)
            rect.setBrush(QBrush(ZONE_COLORS[zone_type]))
            rect.setPen(QPen(ZONE_BORDER_COLORS[zone_type], 2, Qt.DashLine))
            rect.setZValue(0)
            self._scene.addItem(rect)

            zlbl = QGraphicsTextItem(zone_label)
            zlbl.setFont(QFont("Segoe UI", 8, QFont.Bold))
            zlbl.setDefaultTextColor(ZONE_BORDER_COLORS[zone_type].darker(120))
            zlbl.setPos(zone_x + 5, zone_y + 2)
            zlbl.setZValue(1)
            self._scene.addItem(zlbl)

        # x_chip_x, x_chip_y — позиция чипа X; Y-чип размещается на 16px ниже
        status_chips = [
            ("helicopter",    240, 305),
            ("vtol",          710, 305),
            ("service_load",  350, 305),
            ("service_grip",  595, 305),
            ("charger",       485, 465),
        ]

        for zone_key, chip_x, chip_y in status_chips:
            for axis in ("x", "y"):
                offset_y = 0 if axis == "x" else 16
                chip = StatusChipItem(axis.upper(), chip_x, chip_y + offset_y, 'gray', w=40, h=14)
                self._scene.addItem(chip)
                self._zone_chips[f"{zone_key}_{axis}"] = chip

        # 2) Чипы состояния оборудования
        #    Зона helicopter (x=20..230, y=100..700) — H-Table, в левой полосе
        equip_def = [
            ("h_hatch", "Люк", 25, 180),
            ("h_lift", "Лифт", 25, 198),
            ("h_box_lift", "Бокс", 25, 216),
            ("v_hatch", "Люк", 765, 180),
            ("v_lift", "Лифт", 765, 198),
        ]
        for key, chip_label, chip_x, chip_y in equip_def:
            chip = StatusChipItem(chip_label, chip_x, chip_y, 'gray', w=62, h=14)
            self._scene.addItem(chip)
            self._equip_chips[key] = chip

        # 3) Подписи моделей дронов
        for (drone_x, drone_y, drone_text, drone_color) in self.DRONE_LABELS:
            lbl = QGraphicsTextItem(drone_text)
            lbl.setFont(QFont("Segoe UI", 7))
            lbl.setDefaultTextColor(drone_color.darker(130))
            br = lbl.boundingRect()
            lbl.setPos(drone_x - br.width() / 2, drone_y - br.height())
            lbl.setZValue(1)
            self._scene.addItem(lbl)

        # 4) Рёбра
        TRIM = 20
        for (src, dst) in self.EDGES:
            if src not in self.NODE_LAYOUT or dst not in self.NODE_LAYOUT:
                continue
            src_x, src_y = self.NODE_LAYOUT[src][:2]
            dst_x, dst_y = self.NODE_LAYOUT[dst][:2]
            edge_start, edge_end = QPointF(src_x, src_y), QPointF(dst_x, dst_y)
            length = QLineF(edge_start, edge_end).length()
            if length < 1:
                continue
            trim_ratio = TRIM / length
            trim_start = QPointF(src_x + (dst_x - src_x) * trim_ratio,
                                 src_y + (dst_y - src_y) * trim_ratio)
            trim_end = QPointF(dst_x - (dst_x - src_x) * trim_ratio,
                               dst_y - (dst_y - src_y) * trim_ratio)

            forward_arrow = QGraphicsPathItem(_make_arrow_path(trim_start, trim_end, 7))
            forward_arrow.setPen(PEN_NORMAL)
            forward_arrow.setZValue(5)
            self._scene.addItem(forward_arrow)

            backward_arrow = QGraphicsPathItem(_make_arrow_path(trim_end, trim_start, 7))
            backward_arrow.setPen(PEN_BACK_ARROW)
            backward_arrow.setZValue(5)
            self._scene.addItem(backward_arrow)

            self._edge_items[(src, dst)] = [forward_arrow, backward_arrow]

        # 5) Узлы
        for point_name, (node_x, node_y, display, is_endpoint) in self.NODE_LAYOUT.items():
            if point_name == "pHomePosition":
                color, radius = NODE_HOME_COLOR, 24
            elif is_endpoint:
                color, radius = NODE_ENDPOINT_COLOR, 16
            else:
                color, radius = NODE_BASE_COLOR, 18

            node = NodeItem(
                point_name, display, node_x, node_y,
                radius=radius, color=color,
                is_endpoint=is_endpoint, parent_widget=self,
            )
            self._scene.addItem(node)
            self._nodes[point_name] = node

    # ── Публичный API ─────────────────────────────────────────
    def set_current_position(self, point_name: str):
        """Подсвечивает текущую позицию манипулятора на карте (зелёный)."""
        if self._current_point and self._current_point in self._nodes:
            self._nodes[self._current_point].set_current(False)
        self._current_point = point_name
        if point_name in self._nodes:
            self._nodes[point_name].set_current(True)
            self._info_label.setText(f"Текущая позиция: {point_name}")
            self._info_label.setStyleSheet(GREEN_COLOR)

    def highlight_trajectory(self, src: str, dst: str, traj_name: str = ""):
        """Подсвечивает ребро src↔dst, остальные рёбра приглушает."""
        for (edge_src, edge_dst), edge_arrows in self._edge_items.items():
            is_highlighted = (edge_src == src and edge_dst == dst) or (edge_src == dst and edge_dst == src)
            if is_highlighted:
                for item in edge_arrows:
                    item.setPen(PEN_HL)
                    item.setZValue(7)
            else:
                for item in edge_arrows:
                    item.setPen(PEN_DIM)
                    item.setZValue(4)

        label = traj_name if traj_name else f"{src} → {dst}"
        self._info_label.setText(f"Траектория: {label}")
        self._info_label.setStyleSheet(BEIGE_COLOR)

    def reset_highlight(self):
        """Сбрасывает подсветку всех рёбер."""
        for (src, dst), edge_arrows in self._edge_items.items():
            edge_arrows[0].setPen(PEN_NORMAL)
            edge_arrows[0].setZValue(5)
            edge_arrows[1].setPen(PEN_BACK_ARROW)
            edge_arrows[1].setZValue(5)
        self._info_label.setText("Нажмите на точку для выбора маршрута")
        self._info_label.setStyleSheet(BLUE_COLOR)

    def update_plc_state(self, platform, vt, vtol) -> None:
        """
        Обновляет визуализацию ПЛК-состояния на карте.
          platform : ManipulatorPoints | None
          vt       : VtPoints | None
          vtol     : VtolPoints | None
        """
        self._update_platform_chips(platform)
        self._update_node_blocking(platform)
        self._update_equip_chips(vt, vtol)
        self._update_status_panel(platform, vt, vtol)

    # ── Приватные методы обновления состояния ────────────────
    def _update_platform_chips(self, platform) -> None:
        """Обновляет X/Y чипы зон — показывает, где находится каждая ось платформы."""
        if platform is None:
            for chip in self._zone_chips.values():
                chip.update_state("?", 'gray')
            return

        # ось → зона → бит присутствия
        axis_zone_bits = {
            "x": {
                "helicopter":   platform.x_module_h,
                "vtol":         platform.x_module_v,
                "service_load": platform.x_pos_payload,
                "service_grip": platform.x_pos_grippers,
                "charger":      platform.x_charge_h or platform.x_charge_v,
            },
            "y": {
                "helicopter":   platform.y_module_h,
                "vtol":         platform.y_module_v,
                "service_load": platform.y_pos_payload,
                "service_grip": platform.y_pos_grippers,
                "charger":      platform.y_charge_h or platform.y_charge_v,
            },
        }
        axis_alarm = {"x": platform.x_alarm, "y": platform.y_alarm}

        for chip_key, chip in self._zone_chips.items():
            zone_type, axis = chip_key.rsplit("_", 1)
            here = axis_zone_bits[axis].get(zone_type, False)
            alarm = axis_alarm[axis]
            if here and alarm:
                chip.update_state(f"{axis.upper()} HERE|ALM", 'red')
            elif here:
                chip.update_state(f"{axis.upper()} HERE", 'green')
            elif alarm:
                chip.update_state(f"{axis.upper()} ALM", 'red')
            else:
                chip.update_state(axis.upper(), 'gray')

    def _update_node_blocking(self, platform) -> None:
        """Блокирует (серым) узлы зон, куда платформа ещё не подъехала."""
        for point_name, node in self._nodes.items():
            if point_name == "pHomePosition":
                node.set_blocked(False)
                continue
            if point_name == self._current_point:
                continue  # текущую позицию не трогаем

            if platform is None:
                node.set_blocked(False)
                continue

            mapping = ZONE_BLOCK_MAP.get(point_name)
            if mapping is None:
                node.set_blocked(False)
                continue

            x_attr, y_attr, location = mapping
            x_ok = getattr(platform, x_attr, True)
            y_ok = getattr(platform, y_attr, True)
            if x_ok and y_ok:
                node.set_blocked(False)
            else:
                missing = ("XY" if not x_ok and not y_ok
                           else "X-ось" if not x_ok
                           else "Y-ось")
                node.set_blocked(True, f"Платформа ({missing}) не у {location}")

    def _update_equip_chips(self, vt, vtol) -> None:
        """Обновляет чипы состояния оборудования (люк, лифт, бокс)."""

        def _hatch(opened, closed, alarm):
            if alarm:  return "АВАРИЯ", 'red'
            if opened: return "ОТКРЫТ", 'green'
            if closed: return "ЗАКРЫТ", 'yellow'
            return "—", 'gray'

        def _lift(top, bottom, alarm):
            if alarm:   return "АВАРИЯ", 'red'
            if top:     return "ВЕРХ", 'green'
            if bottom:  return "НИЗ", 'yellow'
            return "—", 'gray'

        if vt is not None:
            chip_text, chip_color = _hatch(vt.hatch_opened, vt.hatch_closed, vt.hatch_alarm)
            if chip := self._equip_chips.get("h_hatch"):   chip.update_state(chip_text, chip_color)
            chip_text, chip_color = _lift(vt.lift_top, vt.lift_bottom, vt.lift_alarm)
            if chip := self._equip_chips.get("h_lift"):    chip.update_state(chip_text, chip_color)
            chip_text, chip_color = _lift(vt.box_lift_top, vt.box_lift_bottom, vt.box_lift_alarm)
            if chip := self._equip_chips.get("h_box_lift"): chip.update_state(chip_text, chip_color)
        else:
            for key in ("h_hatch", "h_lift", "h_box_lift"):
                if chip := self._equip_chips.get(key):
                    chip.update_state("НЕТ СВЯЗИ", 'gray')

        if vtol is not None:
            chip_text, chip_color = _hatch(vtol.hatch_opened, vtol.hatch_closed, vtol.hatch_alarm)
            if chip := self._equip_chips.get("v_hatch"): chip.update_state(chip_text, chip_color)
            chip_text, chip_color = _lift(vtol.lift_top, vtol.lift_bottom, vtol.lift_alarm)
            if chip := self._equip_chips.get("v_lift"):  chip.update_state(chip_text, chip_color)
        else:
            for key in ("v_hatch", "v_lift"):
                if chip := self._equip_chips.get(key):
                    chip.update_state("НЕТ СВЯЗИ", 'gray')

    def _update_status_panel(self, platform, vt, vtol) -> None:
        """Обновляет нижнюю панель статуса ПЛК."""

        def _set(key: str, text: str, style: str):
            lbl = self._status_labels.get(key)
            if lbl:
                lbl.setText(text)
                lbl.setStyleSheet(style)

        def _axis_status(prefix: str, powered: bool, homed: bool, alarm: bool):
            """
            Тревога всегда отображается в тексте; питание/хоминг — отдельно.
            """
            # Цвет/стиль: приоритет alarm > нет питания > нет нуля > OK
            if alarm:
                style = MAP_STATUS_ALM
            elif not powered:
                style = MAP_STATUS_OFF
            elif not homed:
                style = MAP_STATUS_WARN
            else:
                style = MAP_STATUS_OK

            # Текст: тревога + операционное состояние (оба могут быть True)
            parts = []
            if alarm:
                parts.append("АВАРИЯ")
            if not powered:
                parts.append("Выкл")
            elif not homed:
                parts.append("нет нуля")
            else:
                if not alarm:
                    parts.append("OK")
            _set(prefix, f"{prefix.split('_')[0].upper()}: {' | '.join(parts)}", style)

        if platform is not None:
            _axis_status("x_axis", platform.x_powered, platform.x_homed, platform.x_alarm)
            _axis_status("y_axis", platform.y_powered, platform.y_homed, platform.y_alarm)

            # Текущая позиция платформы
            pos_map = {
                "ModuleH": platform.module_h_available,
                "ModuleV": platform.module_v_available,
                "ChargeH": platform.charge_h_available,
                "ChargeV": platform.charge_v_available,
                "Payload": platform.pos_load_available,
                "Grippers": platform.pos_grippers_available,
            }
            current_zone = next((name for name, val in pos_map.items() if val), None)
            has_alarm = platform.x_alarm or platform.y_alarm
            if current_zone and has_alarm:
                _set("platform", f"Поз: {current_zone} | ALM", MAP_STATUS_ALM)
            elif current_zone:
                _set("platform", f"Поз: {current_zone}", MAP_STATUS_OK)
            elif has_alarm:
                _set("platform", "Поз: — | ALM", MAP_STATUS_ALM)
            else:
                _set("platform", "Поз: —", MAP_STATUS_OFF)
        else:
            for key in ("x_axis", "y_axis", "platform"):
                _set(key, f"{key.replace('_', ' ')}: ?", MAP_STATUS_OFF)

        def _hatch_txt(opened, closed, alarm):
            return ("АВАРИЯ" if alarm else "Откр" if opened else "Закр" if closed else "—",
                    MAP_STATUS_ALM if alarm else
                    MAP_STATUS_OK if opened else
                    MAP_STATUS_WARN if closed else MAP_STATUS_OFF)

        def _lift_txt(top_pos, bottom_pos, alarm):
            return ("АВАРИЯ" if alarm else "Верх" if top_pos else "Низ" if bottom_pos else "—",
                    MAP_STATUS_ALM if alarm else
                    MAP_STATUS_OK if top_pos else
                    MAP_STATUS_WARN if bottom_pos else MAP_STATUS_OFF)

        # H-table
        if vt is not None:
            status_text, label_style = _hatch_txt(vt.hatch_opened, vt.hatch_closed, vt.hatch_alarm)
            _set("h_hatch", f"Люк: {status_text}", label_style)
            status_text, label_style = _lift_txt(vt.lift_top, vt.lift_bottom, vt.lift_alarm)
            _set("h_lift", f"Лифт: {status_text}", label_style)
            status_text, label_style = _lift_txt(vt.box_lift_top, vt.box_lift_bottom, vt.box_lift_alarm)
            _set("h_box", f"Бокс: {status_text}", label_style)
        else:
            for key in ("h_hatch", "h_lift", "h_box"):
                _set(key, "нет связи", MAP_STATUS_OFF)

        # V-table
        if vtol is not None:
            status_text, label_style = _hatch_txt(vtol.hatch_opened, vtol.hatch_closed, vtol.hatch_alarm)
            _set("v_hatch", f"Люк: {status_text}", label_style)
            status_text, label_style = _lift_txt(vtol.lift_top, vtol.lift_bottom, vtol.lift_alarm)
            _set("v_lift", f"Лифт: {status_text}", label_style)
        else:
            for key in ("v_hatch", "v_lift"):
                _set(key, "нет связи", MAP_STATUS_OFF)

    # ── Внутренние обработчики ────────────────────────────────
    def _on_node_clicked(self, point_name: str):
        self._info_label.setText(f"Выбрана точка: {point_name}")
        self._info_label.setStyleSheet(GREEN_COLOR)

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self._view.scale(factor, factor)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._view.fitInView(
            self._scene.sceneRect().adjusted(-30, -30, 30, 30),
            Qt.KeepAspectRatio)
