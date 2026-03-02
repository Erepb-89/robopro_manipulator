"""
Интерактивная карта траекторий манипулятора ЭРИ Порта.
Визуализация зон, базовых точек и маршрутов перемещения.
"""
from PyQt5.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsTextItem,
    QGraphicsPathItem, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGraphicsItem
)
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QLineF
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QFont, QPainterPath, QPainter
)
import math

# ─── Цвета зон ───────────────────────────────────────────────
from config import GREEN_COLOR, BEIGE_COLOR, BLUE_COLOR, ALABASTER_COLOR

ZONE_COLORS = {
    "helicopter": QColor(41, 98, 255, 30),
    "service_load": QColor(255, 152, 0, 30),
    "service_grip": QColor(255, 152, 0, 30),
    "vtol": QColor(76, 175, 80, 30),
    "charger": QColor(156, 39, 176, 30),
}
ZONE_BORDER_COLORS = {
    "helicopter": QColor(41, 98, 255, 180),
    "service_load": QColor(255, 152, 0, 180),
    "service_grip": QColor(255, 152, 0, 180),
    "vtol": QColor(76, 175, 80, 180),
    "charger": QColor(156, 39, 176, 180),
}

# ─── Цвета точек ─────────────────────────────────────────────
NODE_BASE_COLOR = QColor(41, 98, 255)
NODE_ENDPOINT_COLOR = QColor(255, 87, 34)
NODE_HOME_COLOR = QColor(244, 67, 54)
NODE_CURRENT_COLOR = QColor(76, 175, 80)
NODE_HOVER_COLOR = QColor(255, 193, 7)

# ─── Перья рёбер ─────────────────────────────────────────────
_PEN_NORMAL = QPen(QColor(100, 100, 100, 180), 1.8)
_PEN_DIM = QPen(QColor(180, 180, 180, 70), 1.0, Qt.DotLine)
_PEN_HL = QPen(QColor(255, 152, 0, 240), 3.5)  # оранжевый highlight


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

        self.setPos(x, y)
        self.setZValue(10)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        # QGraphicsDropShadowEffect + setScale() → краш в PyQt5, не используем
        self.setBrush(QBrush(color))
        self.setPen(QPen(color.darker(140), 2))

        # Метка внутри круга (child → не добавляется/удаляется из сцены)
        self._label = QGraphicsTextItem(display_name, self)
        self._label.setFont(QFont("Segoe UI", 7, QFont.Bold))
        self._label.setDefaultTextColor(Qt.white)
        br = self._label.boundingRect()
        self._label.setPos(-br.width() / 2, -br.height() / 2)

        # Тултип — child item, скрытый по умолчанию
        # visible=True/False вместо addItem/removeItem → нет обращений к scene()
        self._tooltip = QGraphicsTextItem(point_name, self)
        self._tooltip.setFont(QFont("Segoe UI", 8))
        self._tooltip.setDefaultTextColor(QColor(30, 30, 30))
        self._tooltip.setZValue(100)
        self._tooltip.setVisible(False)
        tbr = self._tooltip.boundingRect()
        self._tooltip.setPos(-tbr.width() / 2, -radius - tbr.height() - 4)

    def hoverEnterEvent(self, event):
        self.setBrush(QBrush(NODE_HOVER_COLOR))
        self.setPen(QPen(NODE_HOVER_COLOR.darker(150), 3))
        self._tooltip.setVisible(True)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self.setBrush(QBrush(self.base_color))
        self.setPen(QPen(self.base_color.darker(140), 2))
        self._tooltip.setVisible(False)
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.parent_widget:
            self.parent_widget.node_clicked.emit(self.point_name)
        super().mousePressEvent(event)

    def set_current(self, is_current: bool):
        """Подсвечивает узел как текущую позицию манипулятора."""
        if is_current:
            self.base_color = NODE_CURRENT_COLOR
            self.setBrush(QBrush(NODE_CURRENT_COLOR))
            self.setPen(QPen(NODE_CURRENT_COLOR.darker(140), 3))
            self.setScale(1.3)
        else:
            color = NODE_ENDPOINT_COLOR if self.is_endpoint else NODE_BASE_COLOR
            self.base_color = color
            self.setBrush(QBrush(color))
            self.setPen(QPen(color.darker(140), 2))
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
    Клик по узлу: если текущая позиция известна — ищет и подсвечивает
    прямую траекторию до выбранного узла.
    """
    node_clicked = pyqtSignal(str)

    NODE_LAYOUT = {
        "pHomePosition": (500, 400, "Home", False),

        "pHelicopterModule": (200, 400, "pH", False),
        "pHelicopter1": (120, 250, "pH1", False),
        "pHelicopter1Load": (120, 150, "pH1L", True),
        "pHelicopter2": (120, 550, "pH2", False),
        "pHelicopter2Load": (120, 650, "pH2L", True),

        "pLoad": (370, 250, "pL", False),
        "pLoad1": (310, 150, "pL1", True),
        "pLoad2": (430, 150, "pL2", True),

        "pGrippers": (500, 250, "pG", False),
        "pGrippers1": (440, 150, "pG1", True),
        "pGrippers2": (560, 150, "pG2", True),

        "pCharger": (420, 600, "pC", False),
        "pCharger1": (370, 700, "pC1", True),
        "pCharger2": (470, 700, "pC2", True),

        "pVTOLModule": (800, 400, "pV", False),
        "pVTOL1": (880, 250, "pV1", False),
        "pVTOL1Battery": (830, 150, "pV1B", True),
        "pVTOL1Load": (930, 150, "pV1L", True),
        "pVTOL2": (880, 550, "pV2", False),
        "pVTOL2Battery": (880, 650, "pV2C", True),
    }

    EDGES = [
        ("pHomePosition", "pHelicopterModule"),
        ("pHomePosition", "pLoad"),
        ("pHomePosition", "pGrippers"),
        ("pHomePosition", "pCharger"),
        ("pHomePosition", "pVTOLModule"),

        ("pHelicopterModule", "pHelicopter1"),
        ("pHelicopterModule", "pHelicopter2"),
        ("pHelicopter1", "pHelicopter1Load"),
        ("pHelicopter2", "pHelicopter2Load"),

        ("pLoad", "pLoad1"),
        ("pLoad", "pLoad2"),

        ("pGrippers", "pGrippers1"),
        ("pGrippers", "pGrippers2"),

        ("pCharger", "pCharger1"),
        ("pCharger", "pCharger2"),

        ("pVTOLModule", "pVTOL1"),
        ("pVTOLModule", "pVTOL2"),
        ("pVTOL1", "pVTOL1Battery"),
        ("pVTOL1", "pVTOL1Load"),
        ("pVTOL2", "pVTOL2Battery"),
    ]

    ZONES = [
        (50, 100, 180, 600, "helicopter", "Модуль обслуживания\nвертолёта"),
        (280, 100, 150, 120, "service_load", "Зона полезной\nнагрузки"),
        (410, 100, 180, 120, "service_grip", "Зона захватов"),
        (310, 550, 230, 200, "charger", "Зарядная станция"),
        (780, 100, 200, 600, "vtol", "Модуль обслуживания\nВТОЛ"),
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
        # (src, dst) → [forward_item, backward_item]  — для highlight_trajectory
        self._edge_items: dict[tuple, list] = {}
        self._init_ui()

    # ── UI ────────────────────────────────────────────────────
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        title = QLabel("Схема траекторий манипулятора ЭРИ Порта")
        title.setFont(QFont("Segoe UI", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        legend = QHBoxLayout()
        legend.addStretch()
        for lbl, clr in [
            ("Home", NODE_HOME_COLOR),
            ("Базовая точка", NODE_BASE_COLOR),
            ("Конечная точка", NODE_ENDPOINT_COLOR),
            ("Текущая позиция", NODE_CURRENT_COLOR),
        ]:
            dot = QLabel("●")
            dot.setFont(QFont("Segoe UI", 14))
            dot.setStyleSheet(f"color: {clr.name()};")
            txt = QLabel(lbl)
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

        self._build_scene()
        self._view.fitInView(
            self._scene.sceneRect().adjusted(-30, -30, 30, 30),
            Qt.KeepAspectRatio)

        self.node_clicked.connect(self._on_node_clicked)

    # ── Сцена ─────────────────────────────────────────────────
    def _build_scene(self):
        self._scene.clear()
        self._nodes.clear()
        self._edge_items.clear()

        # 1) Зоны
        for (x, y, w, h, zone_type, label) in self.ZONES:
            rect = QGraphicsRectItem(x, y, w, h)
            rect.setBrush(QBrush(ZONE_COLORS[zone_type]))
            rect.setPen(QPen(ZONE_BORDER_COLORS[zone_type], 2, Qt.DashLine))
            rect.setZValue(0)
            self._scene.addItem(rect)
            zlbl = QGraphicsTextItem(label)
            zlbl.setFont(QFont("Segoe UI", 8, QFont.Bold))
            zlbl.setDefaultTextColor(ZONE_BORDER_COLORS[zone_type].darker(120))
            zlbl.setPos(x + 5, y + 2)
            zlbl.setZValue(1)
            self._scene.addItem(zlbl)

        # 2) Подписи моделей
        for (x, y, text, color) in self.DRONE_LABELS:
            lbl = QGraphicsTextItem(text)
            lbl.setFont(QFont("Segoe UI", 7))
            lbl.setDefaultTextColor(color.darker(130))
            br = lbl.boundingRect()
            lbl.setPos(x - br.width() / 2, y - br.height())
            lbl.setZValue(1)
            self._scene.addItem(lbl)

        # 3) Рёбра — сохраняем ссылки для highlight_trajectory
        TRIM = 20
        for (src, dst) in self.EDGES:
            if src not in self.NODE_LAYOUT or dst not in self.NODE_LAYOUT:
                continue
            sx, sy = self.NODE_LAYOUT[src][:2]
            dx, dy = self.NODE_LAYOUT[dst][:2]
            p1, p2 = QPointF(sx, sy), QPointF(dx, dy)
            length = QLineF(p1, p2).length()
            if length < 1:
                continue
            r = TRIM / length
            tp1 = QPointF(sx + (dx - sx) * r, sy + (dy - sy) * r)
            tp2 = QPointF(dx - (dx - sx) * r, dy - (dy - sy) * r)

            fwd = QGraphicsPathItem(_make_arrow_path(tp1, tp2, 7))
            fwd.setPen(_PEN_NORMAL)
            fwd.setZValue(5)
            self._scene.addItem(fwd)

            bwd = QGraphicsPathItem(_make_arrow_path(tp2, tp1, 7))
            bwd.setPen(QPen(QColor(100, 100, 100, 120), 1.5))
            bwd.setZValue(5)
            self._scene.addItem(bwd)

            self._edge_items[(src, dst)] = [fwd, bwd]

        # 4) Узлы
        for point_name, (x, y, display, is_ep) in self.NODE_LAYOUT.items():
            if point_name == "pHomePosition":
                color, radius = NODE_HOME_COLOR, 24
            elif is_ep:
                color, radius = NODE_ENDPOINT_COLOR, 16
            else:
                color, radius = NODE_BASE_COLOR, 18

            node = NodeItem(
                point_name, display, x, y,
                radius=radius, color=color,
                is_endpoint=is_ep, parent_widget=self,
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
        """
        Подсвечивает ребро src↔dst, остальные рёбра приглушает.
        traj_name используется для подписи в info label.
        """
        for (esrc, edst), items in self._edge_items.items():
            is_hl = (esrc == src and edst == dst) or (esrc == dst and edst == src)
            if is_hl:
                for item in items:
                    item.setPen(_PEN_HL)
                    item.setZValue(7)
            else:
                for item in items:
                    item.setPen(_PEN_DIM)
                    item.setZValue(4)

        label = traj_name if traj_name else f"{src} → {dst}"
        self._info_label.setText(f"Траектория: {label}")
        self._info_label.setStyleSheet(BEIGE_COLOR)

    def reset_highlight(self):
        """Сбрасывает подсветку всех рёбер в нормальное состояние."""
        for (src, dst), items in self._edge_items.items():
            items[0].setPen(_PEN_NORMAL)
            items[0].setZValue(5)
            items[1].setPen(QPen(QColor(100, 100, 100, 120), 1.5))
            items[1].setZValue(5)
        self._info_label.setText("Нажмите на точку для выбора маршрута")
        self._info_label.setStyleSheet(BLUE_COLOR)

    # ── Внутренние обработчики ────────────────────────────────
    def _on_node_clicked(self, point_name: str):
        # Базовая реакция: просто показываем имя точки
        # Логика поиска траектории — в MainWindow через node_clicked сигнал
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
