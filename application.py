
import sys

from PySide6 import QtCore, QtGui, QtWidgets

from PySide6.QtCore import Slot, QRectF, Qt, QLineF
from PySide6.QtGui import QAction, QKeySequence, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import QMainWindow, QApplication, QLabel, QPushButton, QGraphicsScene, QGraphicsView, QGraphicsObject, QGraphicsItem, QStyleOptionGraphicsItem, QHBoxLayout, QWidget

import numpy as np

import main

class Vertex(QGraphicsObject):
    def __init__(self, id, x_coord, y_coord, radius = 25, displayed = False, id_visible = True) -> None:
        super().__init__()

        self.__name__ = 'Vertex'
        self.id = id
        self.radius = radius
        self.x_coord = x_coord
        self.y_coord = y_coord
        
        self.color = "#f3f6f4"
        self.id_visible = id_visible

        self.displayed = displayed
        if self.displayed == True:
            self.setZValue(1)
        else:
            self.setZValue(-5)

        self.edges = []
        
        self.setPos(x_coord,y_coord)

        self.canMove = True        # default behavior for mouse dragging
        self.setFlag(QGraphicsItem.ItemIsMovable, enabled = self.canMove)

        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges)
    
    def toggleVisibility(self):
        if self.displayed == True:
            self.displayed = False
            self.setZValue(-5)
        else:
            self.displayed = True
            self.setZValue(1)
        self.updateEdgeVisibility()

    def turnVisible(self):
        self.displayed = True
        self.setZValue(1)
        self.updateEdgeVisibility()
        
    def updateEdgeVisibility(self):
        for edge in self.edges:
            if edge[1].displayed and self.displayed:
                edge[0].displayed = True
            else:
                edge[0].displayed = False


    def moveVertex(self, x, y):
        if main.printing_mode:
            print("node", self.id, "is currently at", self.pos())
            print("moving node",self.id,"from",self.x_coord,self.y_coord,"to",x,y)
        
        self.setPos(x, y)
        self.x_coord = x
        self.y_coord = y
        self.update_edges()

    def addEdge(self, edge):
        self.edges.append(edge)
        edge[0].displayed = self.displayed

        
    def boundingRect(self) -> QRectF:
        return QRectF(0,0,self.radius*2,self.radius*2)        


    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: QWidget = None):
        if self.displayed:
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
            if self.id_visible:
                painter.drawText(self.boundingRect(), Qt.AlignCenter, self.id)


    def toggle_movability(self):
        self.canMove = not self.canMove
        self.setFlag(QGraphicsItem.ItemIsMovable, enabled = self.canMove)

    def toggle_id_visibility(self):
        self.id_visible = not self.id_visible
        
        
    def update_edges(self):
        for (edge, next) in self.edges:
                edge.calculate_location()
        
    # recalculate edges after change in location
    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.update_edges()

        return super().itemChange(change, value)

class Edge(QGraphicsItem):
    def __init__(self, start: Vertex, end: Vertex, weight, displayed = False) -> None:
        super().__init__()

        self.__name__ = 'Edge'

        self.displayed = displayed
        
        self.start = start
        self.end = end
        self.weight = weight
        
        self.start.addEdge((self, end))
        self.end.addEdge((self, start))

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
        if self.start and self.end and self.displayed:
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
        self.layouts_menu = self.menu.addMenu("Layout")
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
        movability_action.setChecked(True)
        self.actions_menu.addAction(movability_action)

        id_visibility_action = QAction("Toggle Node ID Visibility", self)
        id_visibility_action.triggered.connect(self.vertices_toggle_id_visibility)
        id_visibility_action.setCheckable(True)
        id_visibility_action.setChecked(True)
        self.actions_menu.addAction(id_visibility_action)
        

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

        radial_dfs_regeneration_action = QAction("Generate Radial DFS Tree Layout", self)
        radial_dfs_regeneration_action.triggered.connect(self.regenerate_radial_dfs)
        self.layouts_menu.addAction(radial_dfs_regeneration_action)

        radial_prims_regeneration_action = QAction("Generate Radial Prim's Tree Layout", self)
        radial_prims_regeneration_action.triggered.connect(self.regenerate_radial_prims)
        self.layouts_menu.addAction(radial_prims_regeneration_action)

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
        self.dfs = []
        self.prims = []
        self.vertices = {}
        self.initialize_vertices()
        self.layout = self.default_layout
        self.regenerate()
 
        
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
        elif self.layout == "radial dfs":
            if self.dfs == []:
                self.depth_first_search()
            self.coordinates = main.create_radial_coordinates(width, height, self.dfs)
        elif self.layout == "radial prims":
            if self.prims == []:
                self.prims_algorithm()
            self.coordinates = main.create_radial_coordinates(width, height, self.prims)
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
            self.vertices[vertex_id].turnVisible()
            

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

    def regenerate_radial_dfs(self):
        self.layout = "radial dfs"
        self.regenerate()

    def regenerate_radial_prims(self):
        self.layout = "radial prims"
        self.regenerate()
            
# part of graph initialization:
        
    def initialize_vertices(self):
        for vertex_id in self.adjacency_dict.keys():
            new_vertex = Vertex(vertex_id, 0, 0, radius = self.node_radius)
            self.vertices[vertex_id] = new_vertex
            self.scene.addItem(new_vertex)
        self.initialize_edges()


    def initialize_edges(self):
        for start_id in self.adjacency_dict.keys():        
            for e_tuple in self.adjacency_dict[start_id]:
                end_id, to_create, weight = e_tuple
                if to_create == True:
                    self.scene.addItem(Edge(self.vertices[start_id],self.vertices[end_id], weight))
                    if main.printing_mode:
                        print ("added edge from", start_id, "to", end_id,"with weight",weight)
                    

    def vertices_toggle_movability(self):
        for v in self.scene.items():
            if v.__name__ == 'Vertex':
                v.toggle_movability()

    def vertices_toggle_id_visibility(self):
        for v in self.scene.items():
            if v.__name__ == 'Vertex':
                v.toggle_id_visibility()
        self.scene.update()


    def depth_first_search(self, root = "most connected"):      # time complexity of DFS is O(2E) = O(E)
        if root == "most connected":
            root_id = main.most_connected_node_id
        self.dfs = [(root_id, root_id)]
        visited = [root_id]
        self.depth_first_search_next(self.vertices[root_id], visited)
        if len(self.dfs) < len(self.vertices.keys()):
            print("WARNING: There are", len(self.vertices.keys()) - len(self.dfs),"nodes in the graph that are not connected with the rest, these are currently not displayed")

        if main.printing_mode:
            print("dfs order:",self.dfs)
        return self.dfs
        

    def depth_first_search_next(self, vertex, visited):  
       # self.dfs.append(vertex.id)
        for (edge, next) in vertex.edges:
            if next.id not in visited:
                self.dfs.append((vertex.id, next.id))
                visited.append(next.id)
                self.depth_first_search_next(next, visited)

    def prims_algorithm(self, root = "most connected"):
        if root == "most connected":
            root_id = main.most_connected_node_id
        self.prims = [(root_id, root_id)]
        visited = [root_id] 
        distances = {}
        #distances = queue.PriorityQueue()  maybe change it to priorityqueue after it's done for performance reasons
        for (edge, next) in self.vertices[root_id].edges:
            distances[next.id] = (edge.weight, root_id)     # (distance, parent)

        while (len(self.prims) != len(self.vertices.keys())):       # make sure no duplicates go into self.prims
            if len(distances.keys()) == 0:      # if there are no more vertices to check (dictionary is empty)
                print("WARNING: There are",len(self.vertices.keys()) - len(self.prims),"nodes in the graph that are not connected with the rest, these are currently not displayed")
                break
                #raise ValueError ("There are nodes in the graph that are not connected with the rest")
                # TODO: handle this situation, maybe make a new tree with an unused node as new root

            min_dist = None
            min_node_id = None

            for node_id, (distance, parent) in distances.items():      # find minimum weight among nodes connected to MST
                if (min_dist == None)  or (min_dist > distance):
                    min_dist = distance
                    min_node_id = node_id
            self.prims.append((parent, min_node_id))
            visited.append(min_node_id)

            del distances[min_node_id]

            for (edge, next) in self.vertices[min_node_id].edges:   # add new neighbours of new mst node to checking dictionary
                if next.id not in visited:
                    distances[next.id] = (edge.weight, min_node_id)

        if main.printing_mode:
            print("prims order:", self.prims)
        return self.prims



        


if __name__ == "__main__":

    # Qt Application
    app = QApplication(sys.argv)

    window = MainWindow(main.adjacency_dict, "radial dfs")
    window.show()

    

    sys.exit(app.exec())