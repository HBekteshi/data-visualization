
import sys

from PySide6 import QtCore, QtGui, QtWidgets

from PySide6.QtCore import Slot, QRectF, Qt, QLineF
from PySide6.QtGui import QAction, QKeySequence, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QGraphicsScene, QGraphicsView, QGraphicsObject, QGraphicsItem, QStyleOptionGraphicsItem, QHBoxLayout, QWidget


class Vertex(QGraphicsObject):
    def __init__(self, id, x_coord, y_coord, radius = 25, ) -> None:
        super().__init__()

        self.id = id
        self.radius = radius
        self.x_coord = x_coord
        self.y_coord = y_coord

        self.color = "#f3f6f4"


        self.edges = []


    def addEdge(self, edge):
        self.edges.append(edge)
        
    def boundingRect(self) -> QRectF:
        return QRectF(self.x_coord,self.y_coord, self.radius*2, self.radius*2)

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: QWidget = None):

        painter.setRenderHints(QPainter.Antialiasing)
        painter.setPen(
            QPen(
                QColor(self.color).darker(),
                2,
                Qt.SolidLine,
                Qt.RoundCap,
                Qt.RoundJoin,
            )
        )
        painter.setBrush(QBrush(QColor(self.color)))
        painter.drawEllipse(self.boundingRect())
        painter.setPen(QPen(QColor("black")))
        painter.drawText(self.boundingRect(), Qt.AlignCenter, self.id)

        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)

    # recalculate edges after change in location
    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            for edge in self.edges:
                edge.calculate_location()

        return super().itemChange(change, value)

class Edge(QGraphicsItem):
    def __init__(self, start: Vertex, end: Vertex) -> None:
        super().__init__()

        self.start = start
        self.end = end
        self.start.addEdge(self)
        self.end.addEdge(self)

        self.line = QLineF()
        self.color = "black"
        self.thickness = 2

        self.calculate_location()

    def calculate_location(self):
        self.prepareGeometryChange()
        self.line = QLineF(
            self.start.pos() + self.start.boundingRect().center(),
            self.end.pos() + self.end.boundingRect().center(),
        )
    
    def boundingRect(self) -> QRectF:
        return (
            QRectF(self.line.p1(), self.line.p2())
            .normalized()
        )

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget=None):
        if self.start and self.end:
            painter.setRenderHints(QPainter.Antialiasing)

            painter.setPen(
                QPen(
                    QColor(self.color),
                    self.thickness,
                    Qt.SolidLine,
                    Qt.RoundCap,
                    Qt.RoundJoin,
                )
            )
            painter.drawLine(self.line)
        
            
# right now this just contains the example

vertices = [Vertex("12", 25, 25), Vertex("999", -50,-45)]


edges = [Edge(vertices[0],vertices[1])]

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Graph viewer app")

        # Scene
        self.scene = QGraphicsScene()

         # Menu
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("File")

        # Exit QAction
        exit_action = QAction("Exit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)

        self.file_menu.addAction(exit_action)

         # Status Bar
        self.status = self.statusBar()
        self.status.showMessage("Graph loaded and displayed")

        # Window dimensions
        geometry = self.screen().availableGeometry()
        self.setFixedSize(geometry.width() * 0.7, geometry.height() * 0.7)

        
        # here is where the code will be added to load in all the nodes
        for e in edges:
            self.scene.addItem(e)
        
        for v in vertices:
            self.scene.addItem(v)

        

        self.view = QGraphicsView(self.scene)
        self.view.show()


        # graphics displayed in the center
        self.setCentralWidget(self.view)





if __name__ == "__main__":

    # Qt Application
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    

    sys.exit(app.exec())
