
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
        self.setPos(x_coord,y_coord)

        self.canMove = False        # default behavior for mouse dragging
        
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)
      
    def moveVertex(self, x, y):
        # print("node is currently at", self.pos())
        # print("moving node",self.id,"from",self.x_coord,self.y_coord,"to",x,y)
        
        self.setPos(x, y)
        self.x_coord = x
        self.y_coord = y
        self.update_edges()

    def addEdge(self, edge):
        self.edges.append(edge)
        
    def boundingRect(self) -> QRectF:
        return QRectF(0,0,self.radius*2,self.radius*2)        


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
    def __init__(self, given_adjacency_dict, initial_layout = "random", default_radius = 15) -> None:
        super().__init__()
        self.setWindowTitle("Graph viewer app")

        self.adjacency_dict = given_adjacency_dict

         # Menu
        self.menu = self.menuBar()
        self.file_menu = self.menu.addMenu("File")
        self.actions_menu = self.menu.addMenu("Actions")
        self.layouts_menu = self.menu.addMenu("Layout")

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
        layout_regeneration_action = QAction("Regenerate Layout", self) # will regenerate with same layout
        layout_regeneration_action.triggered.connect(self.regenerate)
        self.layouts_menu.addAction(layout_regeneration_action)

        random_regeneration_action = QAction("Generate Random Layout", self) # will regenerate with random layout
        random_regeneration_action.triggered.connect(self.regenerate_random)
        self.layouts_menu.addAction(random_regeneration_action)

        solar_regeneration_action = QAction("Generate Random Solar Layout", self) # will regenerate with solar layout, random angles
        solar_regeneration_action.triggered.connect(self.regenerate_solar)
        self.layouts_menu.addAction(solar_regeneration_action)

        deterministic_solar_regeneration_action = QAction("Generate Deterministic Solar Layout", self) # will regenerate with solar layout, consistent angles
        deterministic_solar_regeneration_action.triggered.connect(self.regenerate_solar_deterministic)
        self.layouts_menu.addAction(deterministic_solar_regeneration_action)

         # Status Bar
        self.status = self.statusBar()
        self.status.showMessage("Graph loaded and displayed")

        # Window dimensions
        geometry = self.screen().availableSize()
        self.screenwidth = geometry.width() * 0.7
        self.screenheight = geometry.height() * 0.7
        self.setFixedSize(self.screenwidth, self.screenheight)
        
        # random coordinates for now
        self.default_layout = initial_layout
        self.default_radius = default_radius
        self.node_radius = default_radius
        
        # Scene and view
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)

        self.scene.setSceneRect(-self.screenwidth/2 + 25, -self.screenheight/2 + 50, self.screenwidth - 50, self.screenheight - 75)

        
        # Coordinates
        self.generate(self.default_layout)             
        self.vertices = {}
        self.add_to_scene(self.coordinates)
        
        
        self.view.show()

        if main.printing_mode:
            print("window width and height:", self.screenwidth, self.screenheight)
            print("scene width:", self.scene.width(), "scene height:", self.scene.height())
            print("view width:", self.view.width(), "view height:", self.view.height())
        
        # graphics displayed in the center
        self.setCentralWidget(self.view)

    


# layout selector             
    def generate(self, layout):
        width = self.scene.width() - self.node_radius * 2
        height = self.scene.height() - self.node_radius * 2

        self.layout = layout
        if self.layout == "random":
            self.coordinates = main.create_random_coordinates(width, height, self.adjacency_dict)
        elif self.layout == "solar":
            self.coordinates = main.create_solar_coordinates(width, height, self.adjacency_dict)
        elif self.layout == "solar deterministic":
            self.coordinates = main.create_solar_coordinates(width, height, self.adjacency_dict, deterministic = True)
        else:
            print("asked for layout", layout)
            raise ValueError ("Unsupported layout requested")

# recreate the graph
    def regenerate(self):
        if main.printing_mode:
            print("calling regenerate")
            print("asking for layout", self.layout)

        # create new set of coordinates based on the current layout
        self.generate(self.layout)

        # move the vertices to their new positions
        for vertex_id in self.coordinates.keys():
            x,y = self.coordinates[vertex_id]
            if main.printing_mode:
                print("reset vertex",vertex_id,"at x_val",x,"and y_val",-y)
            self.vertices[vertex_id].moveVertex(x,-y)

        self.scene.update()
        if main.printing_mode:
            print("window width and height:", self.screenwidth, self.screenheight)
            print("scene width:", self.scene.width(), "scene height:", self.scene.height())
            print("view width:", self.view.width(), "view height:", self.view.height())
        
    def regenerate_random(self):
        self.layout = "random"
        self.regenerate()

    def regenerate_solar(self):
        self.layout = "solar"
        self.regenerate()

    def regenerate_solar_deterministic(self):
        self.layout = "solar deterministic"
        self.regenerate()
            
# part of graph initialization:
            
    def add_to_scene(self, coordinates):
        self.add_vertices(coordinates)
        self.add_edges()
        
    def add_vertices(self, coordinates):
        for vertex_id in coordinates.keys():
            x,y = coordinates[vertex_id]

            # modifying y to negative y to have the graph treat (0,0) as center instead of top left
            new_vertex = Vertex(vertex_id, x, -y, radius = self.node_radius)
            if main.printing_mode:
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

    window = MainWindow(main.adjacency_dict, "solar deterministic")
    window.show()

    

    sys.exit(app.exec())