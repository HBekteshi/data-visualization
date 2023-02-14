
import sys

from PySide6 import QtCore, QtGui, QtWidgets

from PySide6.QtCore import Slot, QRectF, Qt
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


class Edge(QGraphicsItem):
    def __init__(self, start: Vertex, end: Vertex) -> None:
        super().__init__()

        self.start = start
        self.end = end

# right now this just contains the example
def load_vertices():
    return [Vertex("12", 25, 25), Vertex("999", -50,-45)]


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Graph viewer app")

        # Scene
        self.scene = QGraphicsScene()
#        self.graphicsview = QtWidgets.QGraphicsview(scene)


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
        vertices = load_vertices()
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
