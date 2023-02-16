
import sys

from PySide6 import QtCore, QtGui, QtWidgets

from PySide6.QtCore import Slot, QRectF, Qt, QLineF
from PySide6.QtGui import QAction, QKeySequence, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QGraphicsScene, QGraphicsView, QGraphicsObject, QGraphicsItem, QStyleOptionGraphicsItem, QHBoxLayout, QWidget

import numpy as np

import main

class Vertex(QGraphicsObject):
    def __init__(self, id, x_coord, y_coord, radius = 25) -> None:
        super().__init__()

        self.__name__ = 'Vertex'
        self.id = id
        self.radius = radius
        self.x_coord = x_coord
        self.y_coord = y_coord

        self.color = "#f3f6f4"


        self.edges = []

        self.canMove = False

        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)

    def moveVertex(self, x, y):
        self.setPos(x, y)
        self.x_coord = x
        self.y_coord = y
        self.update_edges()

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


    def toggle_movability(self):
        self.canMove = not self.canMove
        self.setFlag(QGraphicsItem.ItemIsMovable, enabled = self.canMove)
        
    def update_edges(self):
        for edge in self.edges:
                edge.calculate_location()
        
    # recalculate edges after change in location
    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.update_edges()

        return super().itemChange(change, value)

class Edge(QGraphicsItem):
    def __init__(self, start: Vertex, end: Vertex) -> None:
        super().__init__()

        self.__name__ = 'Edge'

        self.start = start
        self.end = end
        
        self.start.addEdge(self)
        self.end.addEdge(self)

        self.setZValue(-0.5)

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
        



class MainWindow(QMainWindow):
    def __init__(self, given_adjacency_dict) -> None:
        super().__init__()
        self.setWindowTitle("Graph viewer app")

        self.adjacency_dict = given_adjacency_dict

        

        # Scene
        self.scene = QGraphicsScene()

         # Menu
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("File")
        self.actions_menu = self.menu.addMenu("Actions")

        # Exit QAction
        exit_action = QAction("Exit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)

        self.file_menu.addAction(exit_action)

        # manual movement
        movability_action = QAction("Toggle Manual Movement", self)
        movability_action.triggered.connect(self.vertices_toggle_movability)
        movability_action.setCheckable(True)
        self.actions_menu.addAction(movability_action)

        

            # graph regeneration
        random_regeneration_action = QAction("Regenerate Random Graph", self)
        random_regeneration_action.triggered.connect(self.regenerate)
        # self.actions_menu.addAction(random_regeneration_action)
        self.actions_menu.addAction(random_regeneration_action)


         # Status Bar
        self.status = self.statusBar()
        self.status.showMessage("Graph loaded and displayed")

        # Window dimensions
        geometry = self.screen().availableSize()
        self.screenwidth = geometry.width() * 0.7
        self.screenheight = geometry.height() * 0.7
        self.setFixedSize(self.screenwidth, self.screenheight)
        #self.scene.setSceneRect(-self.screenwidth/2, -self.screenheight/2, self.screenwidth, self.screenheight)
        #print(self.scene.sceneRect)
        
        # random coordinates for now
        self.default_layout = "random"
        self.default_radius = 25

        self.generate(self.default_layout)

        self.vertices = {}

        self.add_to_scene(self.coordinates)


        self.view = QGraphicsView(self.scene)
        self.view.show()


        # graphics displayed in the center
        self.setCentralWidget(self.view)


# layout selector             
    def generate(self, layout):
        self.layout = layout
        if self.layout == "random":
            self.coordinates = main.create_random_coordinates(width = self.screenwidth - self.default_radius * 2, height = self.screenheight - self.default_radius * 2)
        else:
            print("asked for layout", layout)
            raise ValueError ("Unsupported layout requested")

# recreate the graph
    def regenerate(self):
        print("calling regenerate")
        #self.layout = layout
        print("asking for layout", self.layout)
        self.generate(self.layout)
        for vertex_id in self.coordinates.keys():
            x,y = self.coordinates[vertex_id]
            print("reset vertex",vertex_id,"at x_val",x,"and y_val",y)
            self.vertices[vertex_id].moveVertex(x,y)
        

            
# part of graph initialization:
            
    def add_to_scene(self, coordinates):
        self.add_vertices(coordinates)
        self.add_edges()
        
    def add_vertices(self, coordinates):
        for vertex_id in coordinates.keys():
            x,y = coordinates[vertex_id]

            # modifying y to negative y to have the graph treat (0,0) as center instead of top left
            new_vertex = Vertex(vertex_id, x, -y, radius = self.default_radius)
            print("set vertex",vertex_id,"at x_val",x,"and y_val",y)
            self.vertices[vertex_id] = new_vertex
            self.scene.addItem(new_vertex)

    def add_edges(self):
        for start_id in self.adjacency_dict.keys():        
            for e_tuple in self.adjacency_dict[start_id]:
                end_id, to_create = e_tuple
                if to_create == True:
                    self.scene.addItem(Edge(self.vertices[start_id],self.vertices[end_id]))
                    

    def vertices_toggle_movability(self):
        for v in self.scene.items():
            if v.__name__ == 'Vertex':
                v.toggle_movability()





if __name__ == "__main__":

    # Qt Application
    app = QApplication(sys.argv)

    window = MainWindow(main.adjacency_dict)
    window.show()

    

    sys.exit(app.exec())