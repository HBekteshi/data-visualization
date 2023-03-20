
import sys
from collections import deque

from PySide6.QtCore import QRectF, Qt, QLineF, QPointF
from PySide6.QtGui import QAction, QKeySequence, QPainter, QPen, QColor, QBrush, QPolygonF, QPainterPath
from PySide6.QtWidgets import QMainWindow, QApplication, QGraphicsScene, QGraphicsView, QGraphicsObject, QGraphicsItem, QStyleOptionGraphicsItem, QWidget

import numpy as np
import copy

import main

class Vertex(QGraphicsObject):
    def __init__(self, window, id, x_coord, y_coord, radius = 25, displayed = False, id_visible = True) -> None:
        super().__init__()

        self.window = window
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
    
    def mouseReleaseEvent(self, event):
        QGraphicsItem.mouseReleaseEvent(self, event)
        
        x = self.pos().x()
        y = self.pos().y()

        self.x_coord = x
        self.y_coord = -y

        self.window.update_node_position(self.id, self.x_coord, self.y_coord)

        self.window.scene.update()

        #print("dragged node",self.id,"to position",self.x_coord,self.y_coord)

        if self.window.dynamic_forces == True and self.window.layout in ["force bfs", "force random", "force custom"]:
            self.window.layout = "force custom"
            print("updating force layout")
            self.window.regenerate()
        
    def toggleVisibility(self):
        if self.displayed == True:
            self.displayed = False
            self.setZValue(-5)
        else:
            self.displayed = True
            self.setZValue(1)
        self.updateEdgeVisibility()

    def turnVisible(self, edge_update = True):
        self.displayed = True
        self.setZValue(1)
        if edge_update:
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

    def addEdge(self, edge_tuple):          # format (edge object, other vertex object)
        self.edges.append(edge_tuple)
        edge_tuple[0].displayed = self.displayed

        
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
        
        
    def update_edges(self, waypoints = False, radius_change = 0):
        for (edge, next) in self.edges:
                edge.calculate_location()
                if len(edge.waypoints) > 2 and waypoints == True:
    #                print("update edges with waypoints =", waypoints, "; radius change =", radius_change)
                    edge.update_waypoints(edge.waypoints, radius_change, from_outside = False)
        
    # recalculate edges after change in location
    def itemChange(self, change: QGraphicsItem.GraphicsItemChange, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            # x = value.x()
            # y = value.y()
        #    print("scenepos",self.scenePos(),"itemChange value", value, "x and y", x, y)
        #    print("scenerect",self.scene().sceneRect.x(),self.scene().sceneRect.y())
            self.update_edges()
            self.window.scene.update()

        return super().itemChange(change, value)

class Edge(QGraphicsItem):
    def __init__(self, start: Vertex, end: Vertex, weight, displayed = False, segmented = False, directed = False, curved = True) -> None:
        super().__init__()

        self.__name__ = 'Edge'

        self.displayed = displayed
        self.segmented = segmented
        self.curved = curved
        
        self.directed = directed
        if main.G.is_directed() == True:
            self.directed = True

        self.start = start
        self.end = end
        self.weight = weight
        
        self.start.addEdge((self, end))
        self.end.addEdge((self, start))

        self.setZValue(-0.5)

        self.line = QLineF()
        self.color = "black"
        self.thickness = 1

        self.waypoints = [self.start.pos() + self.start.boundingRect().center(), self.end.pos() + self.end.boundingRect().center()]
        
        self.calculate_location()

        self.arrow_size = 10
    
    def update_waypoints(self, waypoints_list, radius_change = 0, from_outside = True):
        self.waypoints = copy.deepcopy(waypoints_list)
        if len(self.waypoints) > 2:
            for count, waypoint in enumerate(self.waypoints):
                if count not in [0, len(self.waypoints)-1]:
                    x_value = waypoint.x()
                    if from_outside:
                        y_value = -waypoint.y() + self.start.window.node_radius
                    else:
                        y_value = waypoint.y() #- radius_change
                    self.waypoints[count] = QPointF(x_value, y_value)

        self.calculate_location(waypoint_update = True)
        self.update()
        
    def toggle_segmentation(self):
        self.segmented = not self.segmented
        self.calculate_location()

    def toggle_curving(self):
        self.curved = not self.curved
        self.calculate_location()

    def calculate_location(self, waypoint_update = False):
        self.prepareGeometryChange()
        if waypoint_update:
            #print("calling calculate location for long")
            starting_point = self.waypoints[0]
            ending_point = self.waypoints[len(self.waypoints)-1]
        else:
            starting_point = self.start.pos() + self.start.boundingRect().center()
            ending_point = self.end.pos() + self.end.boundingRect().center()
            self.waypoints[0] = starting_point
            self.waypoints[len(self.waypoints)-1] = ending_point
        
        self.line = QLineF(
            self.start.pos() + self.start.boundingRect().center(),
            self.end.pos() + self.end.boundingRect().center(),
        )
        if self.segmented == False:
            self.lines = [self.line]
        else:
            self.lines = []
            for count in range(len(self.waypoints)-1):
                line = QLineF(
                    self.waypoints[count],
                    self.waypoints[count+1]
                )
                self.lines.append(line)

        self.buildPath()
        if len(self.waypoints)> 2:
            pass
            # print("for line between nodes",self.start.id,"and",self.end.id)
            # print("self waypoints is:", self.waypoints)
            # print ("self lines is:",self.lines)
        
    # function for building bezier curves, adapted from https://stackoverflow.com/questions/63016214/drawing-multi-point-curve-with-pyqt5   as there's no geometric library for bezier curves in QT
    def buildPath(self):
        factor = 0.25
        waypoints_list = copy.deepcopy(self.waypoints)
        self.path = QPainterPath(waypoints_list[0])
        if len(waypoints_list) == 2:
            self.path.quadTo(waypoints_list[0], waypoints_list[-1])
        else:
            for p, current in enumerate(waypoints_list):
                if p == 0:
                    continue
                elif p == len(waypoints_list) - 1 :
                    break
                # previous segment
                source = QLineF(waypoints_list[p - 1], current)
                # next segment
                target = QLineF(current, waypoints_list[p + 1])
                targetAngle = target.angleTo(source)
                if targetAngle > 180:
                    angle = (source.angle() + source.angleTo(target) / 2) % 360
                else:
                    angle = (target.angle() + target.angleTo(source) / 2) % 360

                revTarget = QLineF.fromPolar(source.length() * factor, angle + 180).translated(current)
                cp2 = revTarget.p2()

                if p == 1:
                    self.path.quadTo(cp2, current)
                else:
                    # use the control point "cp1" set in the *previous* cycle
                    self.path.cubicTo(cp1, cp2, current)

                revSource = QLineF.fromPolar(target.length() * factor, angle).translated(current)
                cp1 = revSource.p2()

            # the final curve, that joins to the last point
            self.path.quadTo(cp1, waypoints_list[-1])
    
    def boundingRect(self) -> QRectF:
        if self.segmented == False:
            return (
                QRectF(self.line.p1(), self.line.p2())
                .normalized()
            )
        else:
            min_y = 9999
            max_y = -9999
            min_x = 9999
            max_x = -9999
            for line in self.lines:
                min_y = min(line.p1().y(), min_y)
                min_x = min(line.p1().x(), min_x)
                max_y = max(line.p2().y(), max_y)
                max_x = max(line.p2().x(), max_x)
            return (
                QRectF(topleft = QPointF(min_x, min_y), bottomright = QPointF(max_x,max_y))
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
            

            if self.directed and self.segmented and self.start.window.check_for_layered_layout():                    # directed and segmented and layered
                if self.curved == True:
                    painter.drawPath(self.path)
                else:
                    for line in self.lines:
                        if line == self.lines[len(self.lines)-1]:
                            start = self.waypoints[len(self.waypoints)-2]
                            end = self.waypoints[len(self.waypoints)-1]
                            self.draw_arrow(painter, start, self.arrow_target(start,end))
                        else:
                            painter.drawLine(line)                    
            elif self.segmented and self.start.window.check_for_layered_layout():                                    # not directed and segmented and layered
                if self.curved == True:
                    painter.drawPath(self.path)
                else:
                    for line in self.lines:
                        painter.drawLine(line)
            elif self.directed:                                     # directed and not segmented
                start = self.line.p1()
                end = self.line.p2()
                self.draw_arrow(painter, start, self.arrow_target(start,end))
            else:                                                   # neither directed nor segmented
                painter.drawLine(self.line) 

                

# function adapted from the QT for Python documentation examples
    def draw_arrow(self, painter: QPainter, start: QPointF, end: QPointF):
        painter.setBrush(QBrush(self.color))

        line = QLineF(end, start)

        angle = np.arctan2(-line.dy(), line.dx())
        arrow_p1 = line.p1() + QPointF(
            np.sin(angle + np.pi / 3) * self.arrow_size,
            np.cos(angle + np.pi / 3) * self.arrow_size,
        )
        arrow_p2 = line.p1() + QPointF(
            np.sin(angle + np.pi - np.pi / 3) * self.arrow_size,
            np.cos(angle + np.pi - np.pi / 3) * self.arrow_size,
        )

        arrow_head = QPolygonF()
        arrow_head.clear()
        arrow_head.append(line.p1())
        arrow_head.append(arrow_p1)
        arrow_head.append(arrow_p2)
        painter.drawLine(line)
        painter.drawPolygon(arrow_head)

# function adapted from the QT for Python documentation examples        
    def arrow_target(self, target, center) -> QPointF:
        #target = self.line.p1()
        #center = self.line.p2()
        radius = self.end.radius
        vector = target - center
        length = np.sqrt(vector.x() ** 2 + vector.y() ** 2)
        if length == 0:
            return target
        normal = vector / length
        target = QPointF(center.x() + (normal.x() * radius), center.y() + (normal.y() * radius))

        return target

        



class MainWindow(QMainWindow):
    def __init__(self, given_adjacency_dict, initial_layout = "random", default_radius = 10) -> None:
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

        non_tree_edge_display_action = QAction("Show Non-Tree Edges", self)
        non_tree_edge_display_action.triggered.connect(self.toggle_nontree_edge_display)
        non_tree_edge_display_action.setCheckable(True)
        non_tree_edge_display_action.setChecked(False)
        self.actions_menu.addAction(non_tree_edge_display_action)

        segmentation_toggle_action = QAction("Toggle Edge Segmentation for Layered Layouts", self)
        segmentation_toggle_action.triggered.connect(self.toggle_edge_segmentation)
        segmentation_toggle_action.setCheckable(True)
        if main.G.is_directed():
            segmentation_toggle_action.setChecked(True)
        self.actions_menu.addAction(segmentation_toggle_action)

        curved_segmentation_toggle_action = QAction("Toggle Edge Curving for Segmented Layout", self)
        curved_segmentation_toggle_action.triggered.connect(self.toggle_edge_curving)
        curved_segmentation_toggle_action.setCheckable(True)
        curved_segmentation_toggle_action.setChecked(True)
        self.actions_menu.addAction(curved_segmentation_toggle_action)

        radius_increase_action = QAction("Increase Node Size",self)
        radius_increase_action.triggered.connect(self.vertices_increase_radius)
        self.actions_menu.addAction(radius_increase_action)

        radius_decrease_action = QAction("Decrease Node Size",self)
        radius_decrease_action.triggered.connect(self.vertices_decrease_radius)
        self.actions_menu.addAction(radius_decrease_action)

        arrow_increase_action = QAction("Increase Arrow Size", self)
        arrow_increase_action.triggered.connect(self.arrow_increase_size)

        arrow_decrease_action = QAction("Decrease Arrow Size", self)
        arrow_decrease_action.triggered.connect(self.arrow_decrease_size)

        if main.G.is_directed() == True:
            self.actions_menu.addAction(arrow_increase_action)
            self.actions_menu.addAction(arrow_decrease_action)

        dynamic_force_layout_action = QAction("Enable Dynamic Forces on Force-Directed Layout",self)
        dynamic_force_layout_action.triggered.connect(self.toggle_dynamic_forces)
        self.actions_menu.addAction(dynamic_force_layout_action)
        dynamic_force_layout_action.setCheckable(True)
        dynamic_force_layout_action.setChecked(True)

        force_custom_regeneration_action = QAction("Apply Force Direction on Current Layout", self)
        force_custom_regeneration_action.setShortcut(Qt.Key_F)
        force_custom_regeneration_action.triggered.connect(self.regenerate_force_custom)
        self.actions_menu.addAction(force_custom_regeneration_action)

            # graph regeneration
        layout_regeneration_action = QAction("Regenerate Layout", self) # will regenerate with same layout
        layout_regeneration_action.triggered.connect(self.regenerate)
        layout_regeneration_action.setShortcut(QKeySequence.Refresh)
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

        radial_bfs_regeneration_action = QAction("Generate Radial BFS Tree Layout", self)
        radial_bfs_regeneration_action.triggered.connect(self.regenerate_radial_bfs)
        self.layouts_menu.addAction(radial_bfs_regeneration_action)

        radial_dfs_regeneration_action = QAction("Generate Radial DFS Tree Layout", self)
        radial_dfs_regeneration_action.triggered.connect(self.regenerate_radial_dfs)
        self.layouts_menu.addAction(radial_dfs_regeneration_action)

        radial_prims_regeneration_action = QAction("Generate Radial Prim's Tree Layout", self)
        radial_prims_regeneration_action.triggered.connect(self.regenerate_radial_prims)
        self.layouts_menu.addAction(radial_prims_regeneration_action)

        force_random_regeneration_action = QAction("Generate Force Directed Random-Initialized Layout", self)
        force_random_regeneration_action.triggered.connect(self.regenerate_force_random)
        self.layouts_menu.addAction(force_random_regeneration_action)

        force_bfs_regeneration_action = QAction("Generate Force Directed BFS-Initialized Layout", self)
        force_bfs_regeneration_action.triggered.connect(self.regenerate_force_bfs)
        self.layouts_menu.addAction(force_bfs_regeneration_action)

        dag_dfs_barycenter_regeneration_action = QAction("Generate DAG DFS-Initialized Layout (Barycenter crossing minimization)", self)
        dag_dfs_barycenter_regeneration_action.triggered.connect(self.regenerate_dag_dfs_barycenter)
        self.layouts_menu.addAction(dag_dfs_barycenter_regeneration_action)

        dag_dfs_median_regeneration_action = QAction("Generate DAG DFS-Initialized Layout (Median crossing minimization)", self)
        dag_dfs_median_regeneration_action.triggered.connect(self.regenerate_dag_dfs_median)
        self.layouts_menu.addAction(dag_dfs_median_regeneration_action)

         # Status Bar
        self.status = self.statusBar()
        self.status.showMessage("Graph loaded and displayed - layout: "+initial_layout)

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

        #self.scene.setSceneRect(-self.screenwidth/2 + 25, -self.screenheight/2 + 50, self.screenwidth - 50, self.screenheight - 75)

        
        # Coordinates
        self.dfs = []
        self.bfs = []
        self.prims = []
        self.vertices = {}
        self.all_edges = {}
        self.tree = None
        self.treetype = None
        self.initialize_vertices()

        # Default Settings
        self.layout = self.default_layout
        self.first_generation = True
        self.display_non_tree_edges = False
        self.dynamic_forces = True
        self.strict_force_binding = True
        self.regenerate()
 
        
        self.view.show()

        if main.printing_mode:
            print("window width and height:", self.screenwidth, self.screenheight)
            print("scene width:", self.scene.width(), "scene height:", self.scene.height())
            print("view width:", self.view.width(), "view height:", self.view.height())
        
        # graphics displayed in the center
        self.setCentralWidget(self.view)

    
    def update_status(self):
        self.status.showMessage("Graph loaded and displayed - layout: "+self.layout)

    def update_node_position(self, node_id, x, y):
        self.coordinates[node_id] = (x,y)

    def check_for_tree_layout(self):
        if self.layout == "force custom":
            return self.tree
        elif self.layout in ["radial dfs","radial bfs", "radial prims"]:
            self.tree = True
            return True
        else:
            self.tree = False
            return False

    def check_for_layered_layout(self):
        if self.layout in ["dag dfs barycenter", "dag dfs median"]:
            return True
        else:
            return False
        
# layout selector             
    def generate(self, layout):
        width = self.screenwidth - 50 - self.node_radius * 2
        height = self.screenheight - 75 - self.node_radius * 2

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
            self.treetype = "dfs"                
            self.coordinates = main.create_radial_coordinates(width, height, self.dfs, self.node_radius)
        elif self.layout == "radial bfs":
            if self.bfs == []:
                self.breadth_first_search()
            self.treetype = "bfs"
            self.coordinates = main.create_radial_coordinates(width, height, self.bfs, self.node_radius)
        elif self.layout == "radial prims":
            if self.prims == []:
                self.prims_algorithm()
            self.treetype = "prims"
            self.coordinates = main.create_radial_coordinates(width, height, self.prims, self.node_radius)
        elif self.layout == "force bfs":
            if self.bfs == []:
                self.breadth_first_search()
            bfs_coords = main.create_radial_coordinates(width, height, self.bfs, self.node_radius)
            self.coordinates = main.create_force_layout_coordinates(width, height, bfs_coords)
        elif self.layout == "force random":
            random_coords = main.create_random_coordinates(width, height, self.adjacency_dict)
            self.coordinates = main.create_force_layout_coordinates(width, height, random_coords)
        elif self.layout == "force custom":
            if self.strict_force_binding == True:
                self.coordinates = main.create_force_layout_coordinates(width, height, self.coordinates, max_iterations=50)
            else:
                self.coordinates = main.create_force_layout_coordinates(self.scene.width(), self.scene.height(), self.coordinates, max_iterations=50)
        elif self.layout == "dag dfs barycenter":
            if self.dfs == []:
                self.depth_first_search()
            self.coordinates, edge_waypoints = main.calc_DAG(width, height, self.dfs, self.adjacency_dict, minimization_method="barycenter")
            self.update_edge_waypoints(edge_waypoints)
        elif self.layout == "dag dfs median":
            if self.dfs == []:
                self.depth_first_search()
            self.coordinates, edge_waypoints = main.calc_DAG(width, height, self.dfs, self.adjacency_dict, minimization_method="median")
            self.update_edge_waypoints(edge_waypoints)
        else:
            print("asked for layout", layout)
            raise ValueError ("Unsupported layout",layout,"requested")

        
    def update_edge_waypoints(self, edge_waypoints):    # key: list of edges where edge is (start_node_id, end_node_id, weight); value: [coords of start, dummy, ..., end]
  #      print("self.all_edges.keys():", self.all_edges.keys())

  #     print("waypoint items are:",edge_waypoints)

        for edge_triple, waypoints_list in edge_waypoints.items():
   #         print("edge triple is",edge_triple,"; waypoints list is",waypoints_list)
            edge_object = self.all_edges[edge_triple]       # key: (start_node_id, end_node_id, weight); value: edge object
            edge_object.update_waypoints(waypoints_list)

   #         print("edge from",edge_object.start.id,"to",edge_object.end.id,"gets waypoint coordinates of",waypoints_list)
    #    print("edge waypoint updating complete")
            
        self.scene.update()


# recreate the graph
    def regenerate(self, same_positions = False):
        if main.printing_mode:
            print("calling regenerate")
            print("asking for layout", self.layout)

        # create new set of coordinates based on the current layout
        if not same_positions:
            self.generate(self.layout)
        
        if (self.first_generation == False and self.check_for_tree_layout() == True):
            for item in self.scene.items():
                item.displayed = False



        # move the vertices to their new positions
        for vertex_id in self.coordinates.keys():
            x,y = self.coordinates[vertex_id]

            if main.printing_mode:
                print("reset vertex",vertex_id,"at x_val",x,"and y_val",-y)

            self.vertices[vertex_id].moveVertex(x,-y)
            
            if self.check_for_tree_layout() and not self.display_non_tree_edges:
                self.vertices[vertex_id].turnVisible(edge_update = False)
            else:
                self.vertices[vertex_id].turnVisible()
            
       # get self.dfs, for parent in selfdfs element [0], check all the edges for next to be element [1], turn on that edge only; make sure update function doesn't update other edges
        if self.check_for_tree_layout() and not self.display_non_tree_edges:
            if self.treetype == "dfs":
                node_list = self.dfs
            elif self.treetype == "bfs":
                node_list = self.bfs
            elif self.treetype == "prims":
                node_list = self.prims
            else:
                raise ValueError ("No selected node list in given tree layout")
        
            for parent_id, child_id in node_list:
        #        print("testing to see if tree displays edge", parent_id,"to",child_id)
                for edge, next in self.vertices[parent_id].edges:
        #            print("node",parent_id,"has edge to:",next.id)
                    if next == self.vertices[child_id]:
                        edge.displayed = True
                        # edge.color = "blue"               # highlighting tree edges
                        # edge.setZValue(-0.3)
                        edge.update()
        #                print("tree displaying edge",parent_id, "to", child_id)

        self.update_status()                
        
        new_bounding_rect = self.scene.itemsBoundingRect()
        self.scene.setSceneRect(new_bounding_rect)
        self.scene.update()
        self.view.updateSceneRect(new_bounding_rect)
        self.first_generation = False

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

    def regenerate_radial_bfs(self):
        self.layout = "radial bfs"
        self.regenerate()
        
    def regenerate_radial_prims(self):
        self.layout = "radial prims"
        self.regenerate()
    
    def regenerate_force_random(self):
        self.layout = "force random"
        self.regenerate()
        
    def regenerate_force_bfs(self):
        self.layout = "force bfs"
        self.regenerate()
    
    def regenerate_force_custom(self):
        self.layout = "force custom"
        self.regenerate()

    def regenerate_dag_dfs_barycenter(self):
        self.layout = "dag dfs barycenter"
        self.regenerate()
            
    def regenerate_dag_dfs_median(self):
        self.layout = "dag dfs median"
        self.regenerate()

# part of graph initialization:
        
    def initialize_vertices(self):
        for vertex_id in self.adjacency_dict.keys():
            new_vertex = Vertex(self, vertex_id, 0, 0, radius = self.node_radius)
            self.vertices[vertex_id] = new_vertex
            self.scene.addItem(new_vertex)
        self.initialize_edges()


    def initialize_edges(self):
        for start_id in self.adjacency_dict.keys():        
            for e_tuple in self.adjacency_dict[start_id]:
                end_id, to_create, weight = e_tuple
                if to_create == True:
                    if main.G.is_directed() == True:
                        new_edge = Edge(self.vertices[start_id],self.vertices[end_id], weight, segmented = True)
                    else:
                        new_edge = Edge(self.vertices[start_id],self.vertices[end_id], weight, segmented = False)
                    self.scene.addItem(new_edge)
                    self.all_edges[(start_id, end_id, weight)] = new_edge    
                    if main.printing_mode:
                        print ("added edge from", start_id, "to", end_id,"with weight",weight)
                    
    def vertices_decrease_radius(self):
        for v in self.vertices.values():
            radius_change = v.radius
            v.radius = max(5, v.radius-5)
            self.node_radius = v.radius
            radius_change = radius_change - v.radius
            v.update_edges(waypoints = True, radius_change = radius_change)        
        self.scene.update()

    def vertices_increase_radius(self):
        for v in self.vertices.values():
            radius_change = v.radius
            v.radius += 5
            self.node_radius = v.radius
            radius_change = radius_change - v.radius
            v.update_edges(waypoints = True, radius_change = radius_change)
        self.scene.update()

    def arrow_increase_size(self):
        for e in self.scene.items():
            if e.__name__ == 'Edge':
                e.arrow_size += 2
        self.scene.update()

    def arrow_decrease_size(self):
        for e in self.scene.items():
            if e.__name__ == 'Edge':
                e.arrow_size = max(0, e.arrow_size - 2)
        self.scene.update()
                    
    def vertices_toggle_movability(self):
        for v in self.scene.items():
            if v.__name__ == 'Vertex':
                v.toggle_movability()

    def vertices_toggle_id_visibility(self):
        for v in self.scene.items():
            if v.__name__ == 'Vertex':
                v.toggle_id_visibility()
        self.scene.update()

    def toggle_dynamic_forces(self):
        self.dynamic_forces = not self.dynamic_forces

    def toggle_nontree_edge_display(self):
        self.display_non_tree_edges = not self.display_non_tree_edges
        if self.check_for_tree_layout() == True:
            self.regenerate(same_positions = True)

    def toggle_edge_segmentation(self):
        for e in self.scene.items():
            if e.__name__ == 'Edge':
                e.toggle_segmentation()
        self.scene.update()

    def toggle_edge_curving(self):
        for e in self.scene.items():
            if e.__name__ == 'Edge':
                e.toggle_curving()
        self.scene.update()

    def depth_first_search(self, root = "most connected"):      # time complexity of DFS is O(2E) = O(E)
        if root == "most connected":
            root_id = main.most_connected_node_id
        self.dfs = [(root_id, root_id)]
        self.max_depth = []
        visited = [root_id]
        self.depth_first_search_next(self.vertices[root_id], visited)
        if len(self.dfs) < len(self.vertices.keys()):
            print("WARNING: There are", len(self.vertices.keys()) - len(self.dfs),"nodes in the",self.layout, "graph that are not connected with the rest, these are currently not displayed")

        if main.printing_mode:
            print("dfs order:",self.dfs)
        return self.dfs
        
    #global max_depth
    def depth_first_search_next(self, vertex, visited):  
       # self.dfs.append(vertex.id)
        for (edge, next) in vertex.edges:
            if next.id not in visited:
                self.dfs.append((vertex.id, next.id))
                visited.append(next.id)
                self.depth_first_search_next(next, visited)

    def breadth_first_search(self, root = "most connected"):
        if root == "most connected":
            root_id = main.most_connected_node_id
        self.bfs = []
        queue = [(root_id, root_id)]
        visited = [root_id]
        while queue:
            vertex_id, parent_id = queue.pop() # remove first vertex of the queue -> pop(0)
            if main.printing_mode:
                print("Vertex_id, parent_id: ", vertex_id, parent_id)
            self.bfs.append((parent_id, vertex_id)) #keep track of the visited vertices
            for (edge, next) in self.vertices[vertex_id].edges:
                if next.id not in visited: #if vertex not visited, append to visited
                    visited.append(next.id)
                    queue.append((next.id, vertex_id)) # its ID and

        if main.printing_mode:
            print("bfs order:", self.bfs)

        if len(self.bfs) < len(self.vertices.keys()):
            print("WARNING: There are", len(self.vertices.keys()) - len(self.bfs),"nodes in the",self.layout, "graph that are not connected with the rest, these are currently not displayed")
        return self.bfs



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
                print("WARNING: There are",len(self.vertices.keys()) - len(self.prims),"nodes in the",self.layout, "graph that are not connected with the rest, these are currently not displayed")
                break
                #raise ValueError ("There are nodes in the graph that are not connected with the rest")
                # TODO: handle this situation, maybe make a new tree with an unused node as new root

            min_dist = None
            min_node_id = None
            min_node_parent = None

            for node_id, (distance, parent) in distances.items():      # find minimum weight among nodes connected to MST
                if (min_dist == None)  or (min_dist > distance):
                    min_dist = distance
                    min_node_id = node_id
                    min_node_parent = parent
            self.prims.append((min_node_parent, min_node_id))
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

    window = MainWindow(main.adjacency_dict, "dag dfs barycenter", default_radius=10)
    #window = MainWindow(main.adjacency_dict, "solar deterministic", default_radius=10)
    window.show()

    

    sys.exit(app.exec())