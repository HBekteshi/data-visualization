import networkx
import pydot
import numpy as np
import scipy
import math
import copy

import queue
import statistics

from PySide6.QtCore import QPointF
from collections import defaultdict

# settings
printing_mode = False
subgraphs_included = False #set to False when loading a graph without subgraphs

#undirected graphs
#G = networkx.Graph(networkx.nx_pydot.read_dot('data/LesMiserables.dot'))
#G = networkx.Graph(networkx.nx_pydot.read_dot('data/JazzNetwork.dot'))
#G = networkx.Graph(networkx.nx_pydot.read_dot('data/rome.dot'))

#directed graphs
G = networkx.DiGraph(networkx.nx_pydot.read_dot('data/noname.dot')) #this is the small directed network
#G = networkx.DiGraph(networkx.nx_pydot.read_dot('data/LeagueNetwork.dot'))

A = pydot.graph_from_dot_file('data/devonshiredebate_withonlytwoclusters.dot')

    

subgraphs = A[0].get_subgraphs()

subgraph_youngest = subgraphs[0]
subgraph_gap = subgraphs[1]
all_edges = A[0].get_edge_list()
inter_layer_edges = []


for edge in all_edges:
    source = edge.get_source()
    destination = edge.get_destination()
    subgraph_youngest_nodes = subgraph_youngest.obj_dict["nodes"]
    subgraph_gap_nodes = subgraph_gap.obj_dict["nodes"]
    #print("source", source)
    #print("destination", destination)
    if source in subgraph_youngest_nodes and destination in subgraph_youngest_nodes:
        subgraph_youngest.add_edge(edge)
        #print("both nodes are in youngest")
    elif source in subgraph_gap_nodes and destination in subgraph_gap_nodes:
        subgraph_gap.add_edge(edge)
        #print("both nodes are in gap")
    else:
        inter_layer_edges.append((source, destination))
        #print("nodes are in gap and youngest")

#print("edges subgraph", subgraph_youngest.get_edges())
#print("edges subgraph", subgraph_gap.get_edges())

G_parent = networkx.DiGraph(networkx.nx_pydot.from_pydot(A[0]))
G_youngest = networkx.DiGraph(networkx.nx_pydot.from_pydot(subgraphs[0]))
G_gap = networkx.DiGraph(networkx.nx_pydot.from_pydot(subgraphs[1]))
subgraphs_list = [G_youngest, G_gap]

if subgraphs_included:              # TODO: fix this temporary measure, so main.G.isdirected() call gets something decent
    G = G_youngest



print("Data loaded")

adjacency_dict_list = []
adjacency_dict = {}
adjacency_dict_sub1 = {}
adjacency_dict_sub2 = {}
adjacency_dict_interlayer = {}
adjacencies = []
adjacencies_sub1 = {}
adjacencies_sub2 = {}
    
def add_nodes_adjacency_dict(adjacency_dict, G):
    """
    receives a list of nodes and initialize it as adjacency dict keys
    """
    for n in G.nodes():
        if printing_mode:
            print(f"Added node with id {n}")
        adjacency_dict[n] = []


# the les miserables import has a node with id '\\n' and no connections

# the first time an edge appears it is marked True, and it will be rendered
# the second time the same edge appears it is marked False, so it will not be rendered
def add_edges_to_adjacency_dict(adjacency_dict, edges):   
    weightcheck = True
    weighted = True
    for e in edges:
        u, v = e
        if weightcheck:
            try:
                weight = G[u][v]["weight"]
            except:
                weighted = False
            finally:
                weightcheck = False
        if weighted:
            weight = G[u][v]["weight"]
            adjacency_dict[u].append((v,True, weight))          # True = will be rendered graphically; False = has already been rendered graphically
            adjacency_dict[v].append((u,False, weight))         # for directed graph, make sure direction is u-->v for True 
        
        else:
            adjacency_dict[u].append((v,True, 1))          # True = will be rendered graphically; False = has already been rendered graphically
            adjacency_dict[v].append((u,False,1))         # for directed graph, make sure direction is u-->v for True 
        if printing_mode:
            print(f"Added edge from {u} to {v}")

max_edges = 0
most_connected_node_id = []

# uncomment if we want to use this again
# least_connected_node_id = None
# average_connected_node_id = None
# random_root_id = np.random.choice(list(adjacency_dict.keys()))

def create_adjacencies(adjacencies, adjacency_dict):
    for id, adj_nodes in list(adjacency_dict.items()): #create dictionary with the size of the number of edges per node
        adjacencies[id] = len(adj_nodes)
    return adjacencies

if(subgraphs_included == False):
    add_nodes_adjacency_dict(adjacency_dict, G)
    add_edges_to_adjacency_dict(adjacency_dict, G.edges())
    adjacencies.append(create_adjacencies(adjacencies_sub1, adjacency_dict))
    most_connected_node_id.append(max(zip(adjacencies[0].values(), adjacencies[0].keys()))[1]) #retrieve id with max adjacency, add to list
    adjacency_dict_list.append(adjacency_dict)
else:
    add_nodes_adjacency_dict(adjacency_dict_sub1, subgraphs_list[0])
    add_edges_to_adjacency_dict(adjacency_dict_sub1, subgraphs_list[0].edges())
    adjacencies.append(create_adjacencies(adjacencies_sub1, adjacency_dict_sub1))
    most_connected_node_id.append(max(zip(adjacencies_sub1.values(), adjacencies_sub1.keys()))[1]) #retrieve id with max adjacency, add to list
    adjacency_dict_list.append(adjacency_dict_sub1)

    add_nodes_adjacency_dict(adjacency_dict_sub2, subgraphs_list[1])
    add_edges_to_adjacency_dict(adjacency_dict_sub2, subgraphs_list[1].edges())
    adjacencies.append(create_adjacencies(adjacencies_sub2, adjacency_dict_sub2))
    most_connected_node_id.append(max(zip(adjacencies_sub2.values(), adjacencies_sub2.keys()))[1]) #retrieve id with max adjacency, add to list
    adjacency_dict_list.append(adjacency_dict_sub2)

    # add all nodes to dict to be able to add interlayer edges, and add the edges
    add_nodes_adjacency_dict(adjacency_dict_interlayer, subgraphs_list[0]) 
    add_nodes_adjacency_dict(adjacency_dict_interlayer, subgraphs_list[1])
    add_edges_to_adjacency_dict(adjacency_dict_interlayer, inter_layer_edges)
    # ignore nodes from dict which have no interlayer edges
    adjacency_dict_interlayer = {id:val for id, val in adjacency_dict_interlayer.items() if val != []}
    # add to adjacency dict list
    adjacency_dict_list.append(adjacency_dict_interlayer)


#print("adjacency_dict_list", adjacency_dict_list)

#print(G.edges('0'))
if printing_mode:
    print("adjacency_dict:",adjacency_dict)

# if we want to apply average and minimum adjacency again, uncomment
# avg_adjacency = round(sum(adjacencies.values()) / len(adjacencies))
# print("average adjacency", avg_adjacency)
# avg_adjacencies_list = [id for id,val in adjacencies.items() if val == avg_adjacency]
# if avg_adjacencies_list == []:
#     average_connected_node_id = None            
#     print("average is None")
# else:
#     average_connected_node_id = np.random.choice(avg_adjacencies_list)
# print("average node", average_connected_node_id)

# min_adjacency = min(adjacencies.values()) + 1
# min_adjacencies_list = [id for id,val in adjacencies.items() if val == min_adjacency]
# if min_adjacencies_list == []:
#     least_connected_node_id = None
# else:
#     least_connected_node_id = np.random.choice(min_adjacencies_list)
#     print("minimum is None")
# print("minimum node", least_connected_node_id)

def create_random_coordinates(width, height, adjacency_dict):
    coordinates = {}
    for n in adjacency_dict.keys():
        x_val = np.random.uniform(-width/2, width/2)
        y_val = np.random.uniform(-height/2, height/2)
        coordinates[n] = (x_val, y_val)
    return coordinates


#random_coordinates = create_random_coordinates(100, 100)

def create_solar_coordinates(width, height, adjacency_dict, deterministic = False, index = 0):
    coordinates = {}

    max_adjacency = max(adjacencies[index].values()) #retrieve the max adjacency
    nr_rings = len(set(adjacencies[index].values())) #calculate # of rings based on the # of unique values in adjacency numbers

    rings_dict = assign_to_rings(adjacencies[index])

    if deterministic == False:
        coordinates = convert_to_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width)
    else:
        coordinates = convert_to_deterministic_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width)

    if printing_mode:
        print("rings_dict - Which nodes belong to which ring")
        print(rings_dict)

    #print("the coordinates of each node")
    #print(coordinates)

    return coordinates

def assign_to_rings(adjacencies):

    #create dictionary with rings, based on the # of rings
    rings_dict = {}
    for ring in set(adjacencies.values()): #create a ring for each unique value in the adjacencies, with that key value as the key
        rings_dict[ring] = []

    #for every node, add it to the ring with the corresponding size id
    for node, size in adjacencies.items(): 
        rings_dict[size].append(node)

    return rings_dict


def calc_radius_solar(height, width, i, nr_rings):
    #calculate the radius of a ring based on the height, width, and the # of rings 
    
    if(height < width):
        radius = (height/2) * (1 - i/nr_rings)         # this is the same as h/2 - i/3 * h/2
    else:
        radius = (width/2) * (1 - i/nr_rings)
    return radius

def polar_to_cartesian(angle, radius, center_x, center_y):
    x_val = center_x + radius * math.cos(angle)
    y_val = center_y + radius * math.sin(angle)
    return (x_val, y_val)   


def convert_to_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width):
    coordinates = {}

    # if the ring with the highest adjacency has just one node, put this one in the middle of the screen.
    if(len(rings_dict[max_adjacency]) == 1):
        max_node_id = rings_dict[max_adjacency][0]
        coordinates[max_node_id] = (0,0)
        nr_rings -= 1
    
        
    # then calculate the radius of the ring and coordinates of the nodes     
    i = 0
    if printing_mode:
        print("ring keys:", rings_dict.keys())
    for ring_id in rings_dict.keys():
        if i == nr_rings:
            break
        radius = calc_radius_solar(height, width, i, nr_rings)
        if printing_mode:
            print("ring", ring_id, "has nodes:", rings_dict[ring_id], "and is given radius",radius)

    #generate random angle for each node --> convert polar coordinates (radius, angle) to cartesian coordinates
        for vertex_id in rings_dict[ring_id]:
            angle = np.random.uniform(0, 2*math.pi) #generate random angle
            coordinates[vertex_id] = polar_to_cartesian(angle, radius,0,0)
            if printing_mode:
                print("assigned to vertex",vertex_id,"a radius of",radius,"and angle of",angle)
        i += 1

    return coordinates


def convert_to_deterministic_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width, angle_change = 0.3):
    coordinates = {}

    # if the ring with the highest adjacency has just one node, put this one in the middle of the screen.

    if(len(rings_dict[max_adjacency]) == 1):
        max_node_id = rings_dict[max_adjacency][0]
        coordinates[max_node_id] = (0,0)
        nr_rings -= 1
    
        
    # then calculate the radius of the ring and coordinates of the nodes     
        
    i = 0
    if printing_mode:
        print("ring keys:", rings_dict.keys())
    for ring_id in rings_dict.keys():
        if i == nr_rings:
            break
        radius = calc_radius_solar(height, width, i, nr_rings)
        if printing_mode:
            print("ring", ring_id, "has nodes:", rings_dict[ring_id], "and is given radius",radius)

    #generate random angle for each node --> convert polar coordinates (radius, angle) to cartesian coordinates
        angle = 0
        for vertex_id in rings_dict[ring_id]:
            angle += angle_change
            #angle = np.random.uniform(0,360) #generate random angle
            coordinates[vertex_id] = polar_to_cartesian(angle, radius,0,0)
            if printing_mode:
                print("assigned to vertex",vertex_id,"a radius of",radius,"and angle of",angle)

        i += 1

    return coordinates

coordinates = {}

def create_radial_coordinates(width, height, node_list, node_radius):
    #coordinates = {}
    annulus_wedge_angles = {} #store each annulus_wedge_angle in here by node_id
    max_depth = get_max_depth(node_list)
    radius_tuple = calc_radius(width, height, max_depth)
    #distance_between_layers = radius_tuple[1]     #play with this number, this is for the distance between the C layers, later make it into width/height function
    distance_between_layers = max(radius_tuple[1], 2*node_radius)
    #start_radius = radius_tuple[0]       # initialize the radius of the first layer, still make this into width/height function
    start_radius = max(radius_tuple[0], 2*node_radius)
    root_node = node_list[0][1]

    if printing_mode:
        print("radius tuple", radius_tuple[0])
        print("radius distance tuple", radius_tuple[1])

    # base case:
    # if the list consists of one node, we put it in the middle of the screen and done
    if(len(node_list) == 1):
        coordinates[root_node] = (0,0)
        return coordinates
    
    #Initialize the root node
    coordinates[root_node] = (0,0)

    # conquer case:
    # first layer after the root
    direct_root_children = calc_direct_children(node_list, root_node)
    nr_root_children = len(direct_root_children)
    if printing_mode:
        print("root children", direct_root_children)
    angle_difference = 2 * math.pi / nr_root_children
    child_angle = 0
    for child in direct_root_children:
        child_id = child[1]
        if printing_mode:
            print("angle of child", child_id, "is", child_angle)

        #determine coordinates child
        parent_x = coordinates[root_node][0]
        parent_y = coordinates[root_node][1]
        coordinates[child_id] = polar_to_cartesian(child_angle, start_radius, parent_x, parent_y)
        children_of_child = calc_direct_children(node_list, child_id)
        if printing_mode:
            print(child_id, "has children", children_of_child)
        parent_angle = child_angle
        child_angle += angle_difference
        #print("children", children_of_child)
        #recurse on children (divide case)
        calc_radial_coordinates_children(node_list, child_id, start_radius, distance_between_layers, parent_angle)

    return coordinates

def calc_ann_wedge(node_list, parent_id, child_id, radius1, radius2):
    #calculate annulus wedge for each vertex based on the formulas from the slides
    length_child = len(calc_all_children(node_list, child_id)) + 1
    length_parent = len(calc_all_children(node_list, parent_id)) + 1
    radius_angle = 2 * math.acos(radius1 / radius2)
    length_angle = length_child / (length_parent-1)
    wedge_angle = min(radius_angle, length_angle)

    if printing_mode:
        print("for", child_id, "radiusangle is", radius_angle,"and length angle", length_angle)

    return wedge_angle

def calc_all_children(node_list, parent_id):
    #calculate all children of a node given its id (parent_id) en the list of all nodes
    all_children = [] #including grandchildren, thus the whole subtree rooted at the parent_id

    for tuple in node_list:
        if tuple[0] in all_children:
            #check if a parent is in the all_children list, if so append its child to the list
            all_children.append(tuple[1])
            #print("add child", tuple[1], "with parent", tuple[0], "to all_children list")
        elif tuple[0] == parent_id and tuple[0] != tuple[1]:
            #check if the node id is not the same as the parent_id and if it is a child of the parent id
            all_children.append(tuple[1])
            #print("add child", tuple[1], "with parent", tuple[0], "to all_children list")
        #else do nothing 
    return all_children

def calc_direct_children(node_list, parent_id):
    direct_children = []
    for tuple in node_list:
        if tuple[0] == parent_id and tuple[0] != tuple[1]:
            #check if the node id is not the same as the parent_id and if it is a direct child of the parent id
            direct_children.append(tuple) #was tuple[1]
            if printing_mode:
                print("add child", tuple[1], "with parent", tuple[0], "to direct_children list")

    return direct_children

def calc_radial_coordinates_children(node_list, parent_node, start_radius, radius_distance, parent_angle):
    #recursive function for calculating radial coordinates
    direct_children = calc_direct_children(node_list, parent_node)
    #print("direct children list in recursion", direct_children)
    if not direct_children:
        #print("child list of parent", parent_node, "is empty")
        return
    
    wedge_angle = calc_ann_wedge(node_list, parent_node, direct_children[0][1], start_radius, start_radius+radius_distance)
    angle_increase = wedge_angle/ (len(direct_children)+1)
    child_angle = parent_angle - (wedge_angle/2) + angle_increase
   
    for child in direct_children:
        child_id = child[1]
        #print("child", child_id, "from parent", child[0])
        #print("angle of child", child_id, "is", child_angle)
        parent_x = coordinates[parent_node][0]
        parent_y = coordinates[parent_node][1]
        coordinates[child_id] = polar_to_cartesian(child_angle, radius_distance, parent_x, parent_y)
        #print("coordinates of child", coordinates[child_id])
        child_radius = start_radius + radius_distance #calculate the radius of the new layer for the new child
        calc_radial_coordinates_children(node_list, child_id, child_radius, radius_distance, child_angle)
        child_angle += angle_increase

node_list_dfs = [('11','11'), ('11','2'), ('2','1'), ('2','3'), ('3','4'), ('2','5'), ('2','6'), ('2','7'), ('2','8'), ('2','9'),
                 ('2','10'), ('11','12'), ('11', '13'), ('13', '24'), ('24', '17'), ('17', '18'), ('18', '27'), ('27', '25'), ('25', '26'),
                 ('26', '28'), ('28', '29'), ('29', '45'), ('29', '46'), ('28', '30'), ('30', '35'), ('35', '36'), ('36', '37')]

def get_node_list_depth_dict(node_list):
    depth_dict = {}
    root_id = node_list[0][0]
    depth_dict[root_id] = 0
    for edge in node_list:
        parent_id, child_id = edge
        depth_dict[child_id] = depth_dict[parent_id] +1
    return depth_dict

def get_max_depth(node_list):
    depth_dict = get_node_list_depth_dict(node_list)
    max_depth = max(depth_dict.values())
    return max_depth

def calc_radius(width, height, max_depth):
    if width < height:
        radius = width / max_depth
        radius_distance = width / (max_depth + 1)
    else:
        radius = height / max_depth
        radius_distance = height / (max_depth + 1)
    return (radius, radius_distance)

def create_force_layout_coordinates(width, height, initial_coords, adjacency_dict, C = 1, max_iterations = 200, index = 0):
    delta = 0.075 # given number within the range (0,1]`
    iteration_count = 0
    coords_dict = initial_coords.copy()
    temp_dict = {key: np.random.uniform(3, 256) for key in list(initial_coords.keys()) } # dictionary with all temperature values
    skew_gauge_dict = {key: 0 for key in list(initial_coords.keys())} # dictionary with all skew gauge values
    prev_force_dict = {key: 0 for key in list(initial_coords.keys())} # init dictionary with prev force values
    area = width * height
    t_min = 3 # minimum temperature
    t_global = 100 #random number for t_global
    
    nr_vertices = len(initial_coords.keys())

    while t_global > t_min and iteration_count < max_iterations: #change max iterations maybe
        coords_dict = force_iteration(width, height, coords_dict, prev_force_dict, temp_dict, skew_gauge_dict, delta, area, nr_vertices, C, adjacency_dict, local_adjacencies = adjacencies[index])
        t_global = sum(temp_dict.values()) / len(temp_dict) #update global temperature
        iteration_count += 1

    return coords_dict


def force_iteration(width, height, old_coordinates_dict, prev_force_dict, temp_dict, skew_gauge_dict,
                    delta_value, area, nr_vertices, C, local_adjacency_dict, local_adjacencies, use_barycenter = True, apply_boundaries = True, single_node_iteration = False):
    new_coordinates_dict = copy.deepcopy(old_coordinates_dict)

    barycenter = [0,0]

    #we can play with these constants
    t_max = 256
    angle_osc = math.pi
    angle_rot = math.pi / 3
    sens_osc = 0.5
    sens_rot = 1 / (2 * nr_vertices)

    if use_barycenter == True:
        for old_coords_tuple in (old_coordinates_dict.values()):
            barycenter[0] += old_coords_tuple[0]
            barycenter[1] += old_coords_tuple[1]
        
        barycenter[0] = barycenter[0] / nr_vertices
        barycenter[1] = barycenter[1] / nr_vertices

    
    if single_node_iteration == True:
        
        raise ValueError ("Single node iteration is not currently supported.")
        # rounds = nr_vertices

        # for i in range(rounds):
        #     # decide which node out of the list (random) --> id is the id of that node
        #     id = np.random.choice(list(old_coordinates_dict.keys()))

        #     coords_tuple = new_coordinates_dict[id]             # (x,y) tuple

        #     force = calc_sum_force(id, coords_tuple, new_coordinates_dict, area, nr_vertices, C, use_barycenter, barycenter, adjacency_dict, local_adjacencies)

        #     new_x = old_coords_tuple[0] + delta_value * force[0]
        #     new_y = old_coords_tuple[1] + delta_value * force[1]

        #     if apply_boundaries == True:
        #         if abs(new_x) > width/2:
        #             if new_x > 0:
        #                 new_x = width/2
        #             else:
        #                 new_x = -width/2
                
        #         if abs(new_y) > height/2:
        #             if new_y > 0:
        #                 new_y = height/2
        #             else:
        #                 new_y = -height/2

        #     new_coordinates_dict[id] = (new_x, new_y)

    else:
   #     print("local adj dict keys is",local_adjacency_dict.keys())
      #  print("old coords dict keys is", old_coordinates_dict.keys())
        for id in (local_adjacency_dict.keys()):
            try:
                old_coords_tuple = old_coordinates_dict[id]             # (x,y) tuple
            except:
              #  print("passing force iteration for node",id)    
                continue
                # print("local adj dict keys is",local_adjacency_dict.keys())
                # print("old coords dict keys is", old_coordinates_dict.keys())
                # print("breaking node id is",id)
                # raise ValueError("breaking node_id is",id)
            #TODO: change global to local adjacencies
            force = calc_sum_force(id, old_coords_tuple, old_coordinates_dict, area, nr_vertices, C, use_barycenter, barycenter, local_adjacency_dict, local_adjacencies)
            prev_force = prev_force_dict[id]

            if prev_force != 0:
                #calculate angle between force and prev force
                unit_vec_force = calc_unit_vec(0,0,force[0],force[1])
                unit_vec_prev_force = calc_unit_vec(0,0,prev_force[0],prev_force[1])
                angle = np.arccos(np.dot(unit_vec_force, unit_vec_prev_force))

                #
                if math.sin(angle) > math.sin(math.pi/2 + angle_rot/2):
                    skew_gauge_dict[id] += sens_rot * np.sign(math.sin(angle))

                if abs(math.cos(angle)) >= math.cos(angle_osc/2):
                    temp_dict[id] += sens_osc * math.cos(angle)

                temp_dict[id] = temp_dict[id] * (1 - abs(skew_gauge_dict[id]))
                temp_dict[id] = min(temp_dict[id], t_max)
                #print("skew gauge of vertex", id, "is", skew_gauge_dict[id])
                #print("temp of vertex", id, "is", temp_dict[id])
                #print("with current force", force, "and previous force", prev_force)

            if force != 0:
                force_magnitude = calc_eucl_dist(0,0,force[0],force[1])
                force = (temp_dict[id] * (force[0] / force_magnitude), temp_dict[id] * (force[1] / force_magnitude))
                new_x = old_coords_tuple[0] + delta_value * force[0]
                new_y = old_coords_tuple[1] + delta_value * force[1]
                barycenter[0] += force[0]
                barycenter[1] += force[1]
            
            if apply_boundaries == True:
                if abs(new_x) > width/2:
                    if new_x > 0:
                        new_x = width/2
                    else:
                        new_x = -width/2
                
                if abs(new_y) > height/2:
                    if new_y > 0:
                        new_y = height/2
                    else:
                        new_y = -height/2

            new_coordinates_dict[id] = (new_x, new_y)
            prev_force_dict[id] = force #store current force as previous force for next round
        # print("node",id,"gets a force push of",force)

    return new_coordinates_dict

def calc_sum_force(current_id, old_coords_tuple, old_coordinates_dict, area, nr_vertices, C, use_barycenter, barycenter, local_adjacency_dict, local_adjacencies, use_mass = True):
    #adj_nodes = calc_direct_children() #to check again          # need node list and parent id  # this only works for a tree structure
    adj_nodes = []

    for edge in local_adjacency_dict[current_id]:
        adj_nodes.append(edge[0])           # appends ID of neighbour vertex

   # print("ideal length of an edge is calculated to be", length)


# attractive forces:
    
    node_mass = 1 + local_adjacencies[current_id]/2
    c_grav = 1/16               # 1 with normal eades, now 1/16 for impulse
    #rand_vec = (np.random.uniform(-32, 32), np.random.uniform(-32, 32)) #randon disturbance vector for force/impuls init
    # [0,0] was original for eades algorithm, initialize impulse p now with all these things
    #force = [(barycenter[0]/nr_vertices - old_coords_tuple[0]) * c_grav * node_mass + rand_vec[0], 
    #         (barycenter[1]/nr_vertices - old_coords_tuple[1]) * c_grav * node_mass + rand_vec[1]]
    force = [0,0]
    length = calc_ideal_length(area, nr_vertices, C)            # unused for eades, used for impulse

    for a_node_id in adj_nodes:
        a_node_coords_tuple = old_coordinates_dict[a_node_id]
        dx, dy = calc_attr_force_eades(128, a_node_coords_tuple, old_coords_tuple) # change 1 to length for fruchterman and vice versa

        if use_mass == True:
            dx = dx / node_mass
            dy = dy / node_mass

        force[0] += dx
        force[1] += dy
    
  # print("with only attractive forces, node", current_id,"gets a force of",force)
        
# repulsive forces:
    for node_id in local_adjacency_dict.keys():
        try:
            if node_id != current_id:
                node_coords = old_coordinates_dict[node_id]

                if node_coords != old_coords_tuple:             # check that the nodes are not in the same location
                #  print("inequality testing", node_coords,old_coords_tuple)
                    dx, dy = calc_rep_force_eades(128, node_coords, old_coords_tuple) #change 1 to length for fruchterman and vice versa
                    force[0] += dx
                    force[1] += dy 
                    
        except:
            #print("passing repulsion on node",node_id)
            continue
            # print("breaking node_id is",node_id)
            # print("local adj dict keys is",local_adjacency_dict.keys())
            # raise ValueError("old coords dict keys is", old_coordinates_dict.keys())
        

# modifications:

    if use_barycenter == True:
       unit_vector_to_bary = calc_unit_vec(old_coords_tuple[0], old_coords_tuple[1], barycenter[0], barycenter[1])
       force[0] += c_grav * node_mass * unit_vector_to_bary[0]
       force[1] += c_grav * node_mass * unit_vector_to_bary[1]



    return force

def calc_ideal_length(area, nr_vertices, C):
    """ 
    input: a parameter C, the number of vertices and area
    output: a float that represent the ideal edge length""" 
    ideal_length = C * math.sqrt(area / nr_vertices)
    return ideal_length

def calc_eucl_dist(x1, y1, x2, y2):
    """ 
    input: x1, y1, x2, y2 coordinates that represent node 1 and node 2 repestively
    output: a float that represent the euclidean distance""" 
    eucl_dist = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
    # if eucl_dist == 0:
    #     print("distance zero:", x1, x2, y1, y2)
    return eucl_dist

def calc_unit_vec(x1, y1, x2, y2):
    """ 
    input: x1, y1, x2, y2 coordinates that represent node 1 and node 2 repestively
    output: a tuple (x,y) for the unit vector""" 
    magnitude = calc_eucl_dist(x1, y1, x2, y2)
    if magnitude == 0:
        return (0,0)
    unit_vecx = (x1 - x2) / magnitude
    unit_vecy = (y1 - y2) / magnitude
    return (unit_vecx, unit_vecy)

def calc_rep_force_fruchter(length, node1, node2):
    """ 
    input: length: ideal length of an edge, float, node1 and node2: a tuple of (x,y) coordinates
    output: a float tuple (x,y) that indicates the repulsive force""" 
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    unit_vec12 = calc_unit_vec(x1, y1, x2, y2)
    eucl_dist = calc_eucl_dist(x1, y1, x2, y2)
    if eucl_dist == 0:
        return (0,0)
    
    rep_forcex = ((length * length) / eucl_dist) * unit_vec12[0]
    rep_forcey = ((length * length) / eucl_dist) * unit_vec12[1]
    
    
    return (rep_forcex, rep_forcey)

def calc_rep_force_eades(length, node1, node2):
    """ 
    input: length: ideal length of an edge, float, node1 and node2: a tuple of (x,y) coordinates
    output: a float tuple (x,y) that indicates the repulsive force""" 
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    unit_vec12 = calc_unit_vec(x1, y1, x2, y2)
    eucl_dist = calc_eucl_dist(x1, y1, x2, y2)
    if eucl_dist == 0:
        return (0,0)
    
    rep_forcex = (length / (eucl_dist * eucl_dist)) * unit_vec12[0]
    rep_forcey = (length / (eucl_dist * eucl_dist)) * unit_vec12[1]
    
    
    return (rep_forcex, rep_forcey)

def calc_attr_force_fruchter(length, node1, node2):
    """
    input: length: ideal length of an edge, float, node1 and node2: a tuple of (x,y) coordinates
    output: a float tuple (x,y) that indicates the attractive force""" 
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    unit_vec12 = calc_unit_vec(x1, y1, x2, y2)
    eucl_dist = calc_eucl_dist(x1, y1, x2, y2)
    if eucl_dist == 0:
        return (0,0)

    attr_forcex = ((eucl_dist * eucl_dist) / length) * unit_vec12[0]
    attr_forcey = ((eucl_dist * eucl_dist) / length) * unit_vec12[1]

    return (attr_forcex, attr_forcey)

def calc_attr_force_eades(length, node1, node2):
    """
    input: length: ideal length of an edge, float, node1 and node2: a tuple of (x,y) coordinates
    output: a float tuple (x,y) that indicates the attractive force""" 
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    unit_vec12 = calc_unit_vec(x1, y1, x2, y2)
    eucl_dist = calc_eucl_dist(x1, y1, x2, y2)
    if eucl_dist == 0:
        return (0,0)

    attr_forcex = 2 * math.log((eucl_dist / length)) * unit_vec12[0]
    attr_forcey = 2 * math.log((eucl_dist / length)) * unit_vec12[1]

    return (attr_forcex, attr_forcey)

def calc_rep_imp(length, chosen_node, node2):
    x1 = chosen_node[0]
    y1 = chosen_node[1]
    x2 = node2[0]
    y2 = node2[1]
    pos_dif = (x1 - x2, y1 - y2)

    if pos_dif[0] != 0 or pos_dif[1] != 0:
        magnitude = calc_eucl_dist(x1, y1, x2, y2)
        rep_forcex = (pos_dif[0] * (length * length)) / (magnitude * magnitude)
        rep_forcey = (pos_dif[1] * (length * length)) / (magnitude * magnitude)
        return(rep_forcex, rep_forcey)
    
    return (0,0)

def calc_attr_imp(length, chosen_node, node2, node_mass):
    x1 = chosen_node[0]
    y1 = chosen_node[1]
    x2 = node2[0]
    y2 = node2[1]
    pos_dif = (x1 - x2, y1 - y2)
    magnitude = calc_eucl_dist(x1, y1, x2, y2)

    attr_forcex = (-pos_dif[0] * (magnitude * magnitude)) / (length * length * node_mass)
    attr_forcey = (-pos_dif[1] * (magnitude * magnitude)) / (length * length * node_mass)

    return (attr_forcex, attr_forcey)

def calc_DAG(width, height, dfs_trees, adjacency_dict, perform_crossing_minimization = True, minimization_method = "median", straighten_edges = True):
    #main function of DAG

    #first remove the cycles in the DAG
    vertex_sequence = create_vertex_seqeuence_eades(adjacency_dict)
    #print("newly created vertex sequence is:", vertex_sequence)
    acyclic_adjacency_dict, reversed_list = reverse_edges(vertex_sequence, adjacency_dict)
    #print("newly created reversed list is",reversed_list)

    #assign vertices to layers --> still have to test this!! but doesn't break anything 
    #print(acyclic_adjacency_dict)
    layer_dict, nodes_per_layer = layer_assignment_dag(dfs_trees, acyclic_adjacency_dict)         
    # layer_dict[node_id] = # of layer of that node;  
    # nodes_per_layer[# of layer] = list of all nodes in that layer

    # create dummy vertices for edges that span multiple layers
    dummy_nodes_per_layer, dummy_adjacency_dict, node_waypoints_ids, dummy_layer_dict = create_dummy_nodes(layer_dict, nodes_per_layer, acyclic_adjacency_dict, reversed_list)
    #print("dummy_nodes_per_layer:", dummy_nodes_per_layer)
    #print("dummy_adjacency_dict:", dummy_adjacency_dict)

    


       # get y-coordinates and initial x-coordinates of vertices
    n_layers = len(dummy_nodes_per_layer)
    layer_height_spacing = height / (n_layers+1)            # could also try just (n_layers)

    x_coords_dict = {}
    y_coords_dict = {}

    # Loop through each layer
    for layer_number, vertices in dummy_nodes_per_layer.items():

        # distribute nodes in a layer equally across the screen width
        layer_spacing = width / (len(vertices) +1)

        # calculate the y-coord for the entire layer
        layer_y_coord = (layer_number + 0.5) * layer_height_spacing

        # Calculate the x-coord and add y-coord for each vertex in the layer
        #print("assigning initial x-coordinates for layer number", layer_number)
        for i, node_id in enumerate(vertices):
            x_coords_dict[node_id] = (i+1) * layer_spacing
            y_coords_dict[node_id] = layer_y_coord
            #print("initial x_coordinate of node",node_id," is now",x_coords_dict[node_id])


    # print("initial x coords dict:", x_coords_dict)

    # print("initial y_coords_dict", y_coords_dict)
            
    # print("acyclic adjacency dict:", acyclic_adjacency_dict)

    #perform iterative crossing minimization
    dummy_nodes_per_layer, dummy_adjacency_dict, x_coords_dict = minimize_crossings(dummy_nodes_per_layer, dummy_adjacency_dict, x_coords_dict, perform_crossing_minimization, minimization_method)

    # the order of layer N is not the order of the list that is gotten by calling dummy_nodes_per_layer[N], it just contains the nodes in that layer
    # dummy nodes have edges' weight == False, real node edges have weights with an integer value
    # relative_positions_in_layer[node_id] = ordinality of this node in its layer (integer value starting from 0)

    #assign coordinates to vertices 
    coordinates = {}
    for node_id in adjacency_dict.keys():
        x_value = x_coords_dict[node_id]
        y_value = y_coords_dict[node_id]
        coordinates[node_id] = (x_value, y_value)
        #print("final x_coordinate of node",node_id," is now",x_coords_dict[node_id])

    
    #reversing back edges that have been changed in the first step with the reversed list made in the reverse_edges function is done within dummy node creation   
    
    edge_waypoints = {}
    
    for edge, waypoints in node_waypoints_ids.items():       # key: (start_node_id, end_node_id, weight); value: [start_node_id, dummy_node1_id, dummy_node2_id, end_node_id]
        edge_waypoints[edge] = []                               # edge_waypoints[same key] = [QPointF(x,y), QPointF(x,y), ..., QPointF(x,y)] = [coords of start, dummy, ..., end]
        for waypoint_id in waypoints:
            x_value = x_coords_dict[waypoint_id]
            y_value = y_coords_dict[waypoint_id]
            edge_waypoints[edge].append(QPointF(x_value, y_value))
#        print(before straightening, "the node",edge[0], "gets an edge with waypoints",edge_waypoints[edge], edge_waypoints[edge])
        

    # straighten the edges
    if straighten_edges:
        for edge, path in edge_waypoints.items():

            if len(path) == 2:
                continue
            straight_positions = np.empty(len(path)-2)
            k = len(path)-2
            dummy_x_coords = np.empty(len(path)-2)

            boundaries = np.empty(len(path)-2, dtype = "object")

            for i, dummy_point in enumerate(path):
                
                if i == 0:
                    continue
                elif i == k+1:
                    break
                else:
                    # add boundaries from neighbours
                    min_bound = -9999
                    max_bound = 9999

                    waypoint_id = node_waypoints_ids[edge][i]
                    waypoint_layer = dummy_layer_dict[waypoint_id]
                    for node_id in dummy_nodes_per_layer[waypoint_layer]:
                        neighbour_x = x_coords_dict[node_id]
                        if node_id == waypoint_id:
                            continue
                        elif neighbour_x < path[i].x() and neighbour_x > min_bound:
                            min_bound = neighbour_x + 0                               # arbitrary buffer
                        elif neighbour_x > path[i].x() and neighbour_x < max_bound:
                            max_bound = neighbour_x
                        else:
                            continue
                
                    boundaries[i-1] = (min_bound, max_bound)

                    # add previous coordinates
                    dummy_x_coords[i-1] = path[i].x()
                    straight_positions[i-1] = path[0].x() + (i/(k+1)) * (path[k+1].x() - path[0].x())      # coordinate of d_i when (u,v) is straight

            original_distance = dummies_distance_sum(dummy_x_coords, straight_positions)

            #print("the optimization attribute of edge from",edge[0],"to", edge[1], "is:",original_distance,"for coordinates",dummy_x_coords)
            if original_distance > 0:
                result = scipy.optimize.minimize(fun = dummies_distance_sum, x0 = dummy_x_coords, args = (straight_positions), bounds = boundaries)

                dummy_x_coords = result.x

                # if result.success == True:
                #     dummy_x_coords = result.x
                # else:
                #     #raise ValueError ("scipy optimize failed, message:",result.message)
                #     dummy_x_coords = result.x
            
            for i, point in enumerate(edge_waypoints[edge]):
                if i == 0:
                    continue
                elif i == k+1:
                    break
                else:
                    point.setX(dummy_x_coords[i-1])

            #print("the optimization attribute of edge from",edge[0],"to", edge[1], "is now:",dummies_distance_sum(dummy_x_coords, straight_positions),"for new coordinates",dummy_x_coords)
        
    return coordinates, edge_waypoints #and something else such that it can read the directions of the edges?

def dummies_distance_sum(dummy_x_coords, straight_positions, method = "absolute"):
    distances_sum = 0
    if method == "quadratic":
        for i in range(len(dummy_x_coords)):
            distances_sum += (dummy_x_coords[i] - straight_positions[i])**2
    elif method == "absolute":
        for i in range(len(dummy_x_coords)):
            distances_sum = np.abs(dummy_x_coords[i] - straight_positions[i])
    elif method == "max":
        for i in range(len(dummy_x_coords)):
            if i == 0:
                max_distance = np.abs(dummy_x_coords[i] - straight_positions[i])
            else:
                max_distance = max(max_distance, np.abs(dummy_x_coords[i] - straight_positions[i]))
        distances_sum = max_distance
    else:
        raise ValueError ("Unsupported method for dummies distance in edge straightening")
    return distances_sum

def calc_outgoing_edges(adjacency_dict):
    #calculates for each vertex how many outgoing edges it has
    outgoing_dict =  {key: 0 for key in list(adjacency_dict.keys())} #initialize to zeros
    for vertex_id in adjacency_dict:
        for triple in adjacency_dict[vertex_id]:
            if triple[1] == True:
                outgoing_dict[vertex_id] += 1
          #      print("The outgoing vertix is", vertex_id, "so we increment by 1")
    
    return outgoing_dict

def calc_incoming_edges(adjacency_dict):
    #calculates for each vertex how many incoming edges it has
    incoming_dict =  {key: 0 for key in list(adjacency_dict.keys())} #initialize to zeros
    for vertex_id in adjacency_dict:
        for triple in adjacency_dict[vertex_id]:
            if triple[1] == True:
                receiving_vertex = triple[0]
                incoming_dict[receiving_vertex] += 1
           #     print("The receiving vertix is", receiving_vertex, "so we increment by 1")

    return incoming_dict

def calc_sink_list(adjacency_dict):
    """
    return a list of vertices which are sinks
    """

    #if it has no outgoing edges, but it has incoming edges, then it is a sink
    incoming_dict =  calc_incoming_edges(adjacency_dict)
    outgoing_dict =  calc_outgoing_edges(adjacency_dict)
    
    #adds a vertex to the sink list if it has incoming edges and no outgoing edges
    sink_list = []
    for vertex_id in incoming_dict:
        if incoming_dict[vertex_id] != 0 and outgoing_dict[vertex_id] == 0:
            sink_list.append(vertex_id)

    return sink_list

def calc_source_list(adjacency_dict):
    """
    return a list of vertices which are sources
    """
    #if it has no incoming edges then it is a source, even if it has no outgoing edges

    incoming_dict = calc_incoming_edges(adjacency_dict)

    source_list = []
    for vertex_id in incoming_dict:
        if incoming_dict[vertex_id] == 0:
            source_list.append(vertex_id)

    return source_list

def create_vertex_seqeuence_eades(adjacency_dict):
    source_list = calc_source_list(adjacency_dict)
    sink_list = calc_sink_list(adjacency_dict)
    vertices = list(adjacency_dict.keys())
    incoming_edges = calc_incoming_edges(adjacency_dict)
    outgoing_edges = calc_outgoing_edges(adjacency_dict)
    vertex_sequence_sinks = []
    vertex_sequence_sources = []

    while vertices:

        for vertex in vertices:
            if vertex in sink_list:
                vertex_sequence_sinks.append(vertex)
                incoming_edges.pop(vertex)
                outgoing_edges.pop(vertex)
                vertices.remove(vertex)

        for vertex in vertices:
            if vertex in source_list:
                vertex_sequence_sources.append(vertex)
                incoming_edges.pop(vertex)
                outgoing_edges.pop(vertex)
                vertices.remove(vertex)

        if vertices:
            u_list = {key: 0 for key in list(incoming_edges.keys())}
            # print("u list before", u_list)
            for vertex in incoming_edges:
                indegree = incoming_edges[vertex]
                outdegree = outgoing_edges[vertex]
                delta = outdegree - indegree
                u_list[vertex] = delta
            u = max(zip(u_list.values(), u_list.keys()))[1]
            # print("u list after", u_list)
            # print("incoming edges", incoming_edges)
            # print("outgoing edges", outgoing_edges)
            # print("max value u", u)
            vertex_sequence_sources.append(u)
            incoming_edges.pop(u)
            outgoing_edges.pop(u)
            vertices.remove(u)
    
    vertex_sequence = vertex_sequence_sources + vertex_sequence_sinks
    return vertex_sequence

    # if weighted:
    #     weight = G[u][v]["weight"]
    #     adjacency_dict[u].append((v,True, weight))          # True = will be rendered graphically; False = has already been rendered graphically
    #     adjacency_dict[v].append((u,False, weight))         # for directed graph, make sure direction is u-->v for True 

def reverse_edges(vertex_sequence, adjacency_dict):
    reversed_list = [] #a list of (x,y) which indicate that the edge between xy has been reversed
    acyclic_adjacency_dict = copy.deepcopy(adjacency_dict)
    for index, vertex in enumerate(vertex_sequence):
        edges = acyclic_adjacency_dict[vertex]
        for edge in edges:
         #   print("testing an edge to reverse", edge)
            if(edge[1] == True):
                receiving_vertex = edge[0]
                receiving_vertex_index = vertex_sequence.index(receiving_vertex)
                if(receiving_vertex_index < index):
             #       print("vertex with index", index, "has an edge pointing back to vertex index", receiving_vertex_index)
                    #reverse the edge
                    weight = edge[2]
                    new_edge_uv = (edge[0], False, weight)
                    new_edge_vu = (vertex, True, weight)
                    old_edge_vu = (vertex, False, weight)
                    reversed_list.append((vertex, edge[0])) #add to the list of edges that have been reversed
                    new_edge_list = edges
                    #delete old entry from dictionary
               #     print("remove edge", edge, "from adjacency dict from", vertex)
                    new_edge_list.remove(edge) #from current vertex
              #      print("remove edge", old_edge_vu, "from adjacency dict from", edge[0])
                    acyclic_adjacency_dict[edge[0]].remove(old_edge_vu) #from receiving vertex
                    #add new entry to dictionary
             #       print("add edge", new_edge_uv, "to adjacency dict from", vertex)
                    new_edge_list.append(new_edge_uv)
                    acyclic_adjacency_dict[vertex] = new_edge_list
            #        print("add edge", new_edge_vu, "to adjacency dict from", edge[0])
                    acyclic_adjacency_dict[edge[0]].append(new_edge_vu)
            
        

    #loop through the vertex sequence
    #check for each vertex if it has an edge that point to a vertex with a lower index in the list
        #use index() function for this
    #if so, reverse that edge
    #if not, continue looping

    return acyclic_adjacency_dict, reversed_list

def layer_assignment_dag(dfs_trees, adjacency_dict):
    """input: the adjacency dict and a dfs list with tuples of (parent_id, node_id)"""
    #print("inside function with start vertex", vertex)
    layer_dict = {key: 0 for key in list(adjacency_dict.keys())}
    to_assign = {key: False for key in list(adjacency_dict.keys())}

    for dfs in dfs_trees:
        print("starting new tree")
        for tuple in dfs:
            start_vertex = tuple[1]
            #layer_dict[start_vertex] = 0
    #      print("inside for with start vertex", start_vertex)
            for edge in adjacency_dict[start_vertex]:
    #         print("inside for with vertex", edge[0], "with start vertex", start_vertex)
                if to_assign[edge[0]] == False:
        #           print("inside if with start vertex", start_vertex)
                    if edge[1] == True:
                        layer_nr = layer_dict[start_vertex] + 1
                        layer_dict[edge[0]] = layer_nr
                        print("vertex", edge[0], "with parent node", start_vertex, "is assigned layer", layer_nr)
                    else:
                        layer_nr = layer_dict[start_vertex] - 1
                        layer_dict[edge[0]] = layer_nr
                        print("vertex", edge[0], "with previous adjacent node", start_vertex, "is assigned layer", layer_nr)
                    to_assign[start_vertex] = True
                    #layer_assignment_dag(dfs, adjacency_dict, to_assign, layer_dict, edge[0], layer_nr)
                else:
                    if printing_mode:
                        print(start_vertex, "is already assigned to layer", layer_dict[start_vertex])

    #ensure minimum value is 0, and increase all values with the difference
    # puts nodes in a dictionary that has all in the nodes in a list with a layer value as key           

    minimum_val = min(layer_dict.values())
    for layer in layer_dict:
        layer_dict[layer] += abs(minimum_val)

    nodes_per_layer = {key: [] for key in set(list(layer_dict.values()))}
    #print("all values layers,", layer_dict.values())
    #print("node layer dict", nodes_per_layer)
    for node in layer_dict:
        layer_value = layer_dict[node]
        nodes_per_layer[layer_value].append(node)

    #print("to assign dict", to_assign)    
    #print("layer dict", layer_dict)
    #print("nodes per layer dict", nodes_per_layer)
    return layer_dict, nodes_per_layer


# step 1: create dummy nodes --> add to layers
# step 2: create comparison function between two adjacent layers (two different functions, median and barycenter)
# step 3: go through the entire graph, first upwards, then downwards
# step 4: count crossings, see if there is improvement --> if not then stop, if yes then back to step 3


def minimize_crossings(dummy_nodes_per_layer, dummy_adjacency_dict, x_coords_dict, perform_crossing_minimization, minimization_method):
    
    number_of_layers = len(dummy_nodes_per_layer.keys())

    # calculate direct layer neighbours of each sequential layer pair
    downwards_neighbours_set = []
    upwards_neighbours_set = []
    downwards_degrees = {}
    upwards_degrees = {}
        
    for layer in range(number_of_layers):
        if layer != 0:
            node_neighbours, upwards_degrees = get_layer_neighbours(dummy_adjacency_dict, dummy_nodes_per_layer[layer-1], dummy_nodes_per_layer[layer], upwards_degrees)
            upwards_neighbours_set.append(node_neighbours)
        if layer != (number_of_layers - 1):
            node_neighbours, downwards_degrees = get_layer_neighbours(dummy_adjacency_dict, dummy_nodes_per_layer[layer + 1], dummy_nodes_per_layer[layer], downwards_degrees)
            downwards_neighbours_set.append(node_neighbours)
        
    downwards_neighbours_set.reverse()         

# iteration: after doing a pair of upward/downward passes, compare number of crossings before and after, if worse, stop and keep previous
    
    # print("upwards degrees are:",upwards_degrees)

    # print("downwards degrees are:",downwards_degrees)

    # initial crossings:
    previous_crossings = 0
    for neighbours_set in upwards_neighbours_set:
        previous_crossings += count_crossings(neighbours_set, x_coords_dict, self_printing_mode=False)

    print("initial crossings count:",previous_crossings)

    while (perform_crossing_minimization == True):
        previous_x_coords = x_coords_dict

        if previous_crossings == 0:
            break

    # one downward pass:
        downwards_crossings = 0
        for neighbours_set in downwards_neighbours_set:
    #        relative_positions_in_layer = permute_layer(neighbours_set, downwards_degrees, relative_positions_in_layer)
            x_coords_dict = permute_layer(neighbours_set, downwards_degrees, x_coords_dict, method = minimization_method)
            downwards_crossings += count_crossings(neighbours_set, x_coords_dict, self_printing_mode = False)


        #print("after downwards pass, the x pos dict is:", x_coords_dict)
        print("after downwards pass, there are",downwards_crossings,"crossings, down from", previous_crossings)
        downwards_x_coords = x_coords_dict
        if downwards_crossings == 0:
            new_crossings = downwards_crossings
            break

        # one upward pass:
        new_crossings = 0
        for neighbours_set in upwards_neighbours_set:
    #     relative_positions_in_layer = permute_layer(neighbours_set, upwards_degrees, relative_positions_in_layer)
            x_coords_dict = permute_layer(neighbours_set, upwards_degrees, x_coords_dict, method = minimization_method)
            new_crossings += count_crossings(neighbours_set, x_coords_dict, self_printing_mode = False)

     #   print("after upwards pass, the x pos dict is:", x_coords_dict)
        print("after upwards pass, there are",new_crossings,"crossings, down from",previous_crossings)

        
        # if the downwards half of the iteration is better, we keep that one
        if downwards_crossings < new_crossings:
            print("moment 1")
            new_crossings = downwards_crossings
            x_coords_dict = downwards_x_coords
        
        # if the iteration is perfect or results in no benefit stop iterating
        if new_crossings == 0:
            break
        elif new_crossings >= previous_crossings:
            print("moment 2")
            x_coords_dict = previous_x_coords
            break
        else:
            print("moment 3")
            previous_crossings = new_crossings
            previous_x_coords = x_coords_dict
    
    if perform_crossing_minimization:
        print("the final crossings count is",new_crossings)
    

    return dummy_nodes_per_layer, dummy_adjacency_dict, x_coords_dict


def count_crossings(node_neighbours, x_coords_dict, self_printing_mode = printing_mode):      # counts current crossings between current layer and previous layer
    crossings = 0
    # if self_printing_mode:
    #     print("node_neighbours_items() is", node_neighbours.items())
    for i, node_tuple in enumerate(node_neighbours.items()):            # check all nodes in current layer --> that node is current_node, its x-coordinate is x1
        current_node_id, neighbours = node_tuple
        # if self_printing_mode:
        #     print("current node is", current_node_id)
        x1 = x_coords_dict[current_node_id]
        for neighbour_node in neighbours:                               # check all neighbours of current node in previous layer --> that node is neighbour_node, its x-coordinate is x2
            # if self_printing_mode:
                # print("looking at neighbour_node", neighbour_node)
                # print("j will be now in this range:", range(i+1, len(node_neighbours.items())))

            x2 = x_coords_dict[neighbour_node]

            for j in range(i+1, len(node_neighbours.items())):            # check all nodes in current layer that haven't yet been checked as 'current node' -->
                other_node_id, other_neighbours = list(node_neighbours.items())[j]             # that node is other_node, its x-coordinate is x3

                # if self_printing_mode:
                    # print("j is", j)
                    # print("list(node_neighbours.items())["+str(j)+"] is", list(node_neighbours.items())[j])
                    # print("looking at other node", other_node_id)

                x3 = x_coords_dict[other_node_id]
                for other_neighbour_id in other_neighbours:                 # check all neighbours of other_node in the previous layer --> that node is other_neighbour, its coordinate is x4
                    # if self_printing_mode:
                        # print("looking at other neighbour", other_neighbour_id)
                    x4 = x_coords_dict[other_neighbour_id]
                
                    if ((x1 > x3 and x2 < x4) or (x1 < x3 and x2 > x4)):            # the edge x1 --- x3 crosses with the edge x2 --- x4 if this statement is true
                        crossings += 1
                        if self_printing_mode:
                            print("there is a crossing between edges",current_node_id,"to",neighbour_node,"and",other_node_id,"to",other_neighbour_id)

    return crossings




def permute_layer(node_neighbours, degrees, x_coords_dict, method = "barycenter", self_printing_mode = False):
    
    #print("relative position in layer", relative_positions_in_layer)
    proposed_positions = queue.PriorityQueue()          # contains items in the form (relative location, node_id), .get() extracts node id with smallest location
    positions_set = set()

    offset_value = 30
    median_offset_value = 35
    
    if method == "barycenter":
        for node_id, neighbours in node_neighbours.items():
            degree = degrees[node_id]
            if degree == 0:
                proposed_position = x_coords_dict[node_id]
                if self_printing_mode:
                    print("proposing node", node_id, "with degree 0 to be in position", proposed_position)
            else:
                sum = 0
    #            print("neighbours", neighbours)
                for neighbour_node in neighbours:
     #               print("neighbour node:",neighbour_node)
      #              print("x coord of neighbour:", x_coords_dict[neighbour_node])
                    sum += x_coords_dict[neighbour_node]
                proposed_position = (sum / degree)
                if self_printing_mode:
                    print("proposing node", node_id, "to be in position", proposed_position)

            
                offset = offset_value                                      # introduce tiny gap in the event of equality
                while proposed_position in positions_set:
                    if (proposed_position + offset) not in positions_set:
                        proposed_position += offset
                        break
                    elif (proposed_position - offset) not in positions_set:
                        proposed_position -= offset
                        break
                    else:
                        if self_printing_mode:
                            print("offsetting position of node", node_id)
                        offset += offset_value

                positions_set.add(proposed_position)

            proposed_positions.put((proposed_position, node_id))
        

        while (proposed_positions.qsize() > 0):
            x_coord, node_id = proposed_positions.get()
            #print("we extract from the pqueue:", node_id)
            x_coords_dict[node_id] = x_coord
            if self_printing_mode:
                print("placing node",node_id,"into position",x_coord)


        return x_coords_dict

    elif method == "median":
        for node_id, neighbours in node_neighbours.items():
            degree = degrees[node_id]
            # print("node",node_id,"has degree",degree)
            if degree == 0:
                proposed_position = 0
            else:
                neighbour_positions = []
                for neighbour_node in neighbours:
                    neighbour_positions.append(x_coords_dict[neighbour_node])
                proposed_position = statistics.median_high(neighbour_positions)                 # in the case of an even number of neighbours, it takes the higher median value
                if self_printing_mode:
                    print("the neighbours list is", neighbour_positions)
                    print("proposing to put node",node_id,"into position",proposed_position)
                
            offset = median_offset_value                                    # introduce tiny gap in the event of equality
            while proposed_position in positions_set:
                if (proposed_position + offset) not in positions_set:
                    proposed_position += offset
                    break
                elif (proposed_position - offset) not in positions_set:
                    proposed_position -= offset
                    break
                else:
                    offset += median_offset_value            

            positions_set.add(proposed_position)

            proposed_positions.put((proposed_position, node_id))

        while (proposed_positions.qsize() > 0):
            x_coord, node_id = proposed_positions.get()
            x_coords_dict[node_id] = x_coord
            if self_printing_mode:
                print("placing node",node_id,"into position",x_coord)

        return x_coords_dict
            

    else:
        raise ValueError ("unsupported layout permutation function requested")


def get_layer_neighbours(dummy_adjacency_dict, previous_layer, current_layer, degrees):

    node_neighbours = {}

    for node in current_layer:
        node_neighbours[node] = []
        edges = dummy_adjacency_dict[node]
        for edge in edges:
            end_node_id = edge[0]
            if (end_node_id in previous_layer) and (end_node_id not in node_neighbours[node]):
                node_neighbours[node].append(end_node_id)
        degrees[node] = len(node_neighbours[node])
                                                     # degrees[node_id] = count of how many neighbouring nodes are in the previous layer
    return node_neighbours, degrees                  # node_neighbours[node_id] = list of neighbouring node_ids that are in the previous layer




def create_dummy_nodes(layer_dict, nodes_per_layer, acyclic_adjacency_dict, reversed_list):

  # the order of layer N is the list that is gotten by calling dummy_nodes_per_layer[N]
    
    dummy_adjacency_dict = copy.deepcopy(acyclic_adjacency_dict)
    dummy_nodes_per_layer = copy.deepcopy(nodes_per_layer)
    dummy_layer_dict = copy.deepcopy(layer_dict)

    node_waypoints_ids = {}             # key: list of edges where edge is (start_node_id, end_node_id, weight); value: [start_node_id, dummy_node1_id, dummy_node2_id, end_node_id]

    #print("reversed list is", reversed_list)

    for start_node_id, edges in acyclic_adjacency_dict.items():
        start_layer = layer_dict[start_node_id]
        for edge in edges:                      # edge is (end_node_id, truth value, weight)
            end_node_id = edge[0]
    
            if (end_node_id, start_node_id) in reversed_list:           # swap truth values back for reversed edges
                truth_value = not edge[1]
                reversed_list.remove((end_node_id, start_node_id))
            else:
                truth_value = edge[1]

            if truth_value == True:                 # we only look at edges that go from a lower layer to a higher layer
                weight = edge[2]

                node_waypoints_ids[(start_node_id, end_node_id, weight)] = [start_node_id]         

                end_node_id = edge[0]
                end_layer = layer_dict[end_node_id]
                layer_difference = end_layer - start_layer
                if layer_difference > 1:
                    for count in range(layer_difference - 1):

                        dummy_layer = start_layer  + count + 1

                        dummy_id = "dummy " + start_node_id + " to " + end_node_id + " in layer " + str(dummy_layer)

                        if count == (layer_difference - 2):
                            target_id = end_node_id
                        else:
                            target_id = "dummy " + start_node_id + " to " + end_node_id + " in layer " + str(dummy_layer + 1)

                        if printing_mode:
                            print("trying to create", dummy_id)
                            
                    
                        if dummy_id not in dummy_nodes_per_layer[dummy_layer]:
                            node_waypoints_ids[(start_node_id, end_node_id, weight)].append(dummy_id)
                            dummy_nodes_per_layer[dummy_layer].append(dummy_id)
                            dummy_layer_dict[dummy_id] = dummy_layer

                            if printing_mode:
                                print("successfully created", dummy_id)
                            
                            if dummy_adjacency_dict.get(dummy_id) == None:
                                dummy_adjacency_dict[dummy_id] = [(target_id, True, False)]        # all dummy nodes have weight False
                            else:
                                dummy_adjacency_dict[dummy_id].append((target_id, True, False))        

                            if dummy_adjacency_dict.get(target_id) == None:
                                dummy_adjacency_dict[target_id] = [(dummy_id, False, False)]
                            else:
                                dummy_adjacency_dict[target_id].append((dummy_id, False, False))

                        else:
                            if printing_mode:
                                print("failed to create", dummy_id)


                node_waypoints_ids[(start_node_id, end_node_id, weight)].append(end_node_id)

    #print("node_waypoints_ids:", node_waypoints_ids)

    return dummy_nodes_per_layer, dummy_adjacency_dict, node_waypoints_ids, dummy_layer_dict


def floyd_warshall_dict(graph):
    """
    input: a networkx graph
    output: a default dictionary with the shortest distance from node to node
    description: the floyd warshall algorithm to compute the shortest distance for weighted graphs
    """
    #default is set as inf if nodes are not connected
    distance = defaultdict(lambda: defaultdict(lambda: float('inf')))

    # node connections with itself are 0
    for node in graph.nodes():
        distance[node][node] = 0

    # retrieve the weight of the graph
    try:
        for e in graph.edges(data=True):
            print("edge", e)
            i, j, data = e
            distance[i][j] = data.get('weight', 1.0)
    except:
        print("this graph is not weighted, use a weighted graph")

    # commmpute the shortest distance
    for k in graph.nodes():
        for i in graph.nodes():
            for j in graph.nodes():
                if distance[i][j] > distance[i][k] + distance[k][j]:
                    distance[i][j] = distance[i][k] + distance[k][j]

    return distance

def floyd_warshall_matrix(graph):
    """
    input: a networkx graph
    output: a default dictionary with the shortest distance from node to node
    description: the floyd warshall algorithm to compute the shortest distance for weighted graphs
    """
    #default is set as inf if nodes are not connected
    nr_vertices = len(graph.nodes())
    distance = np.full((nr_vertices, nr_vertices), np.inf)
    index_node_dict = {}
    node_index_dict = {}

    # node connections with itself are 0
    for index, node in enumerate(graph.nodes()):
        distance[index][index] = 0
        index_node_dict[index] = node
        node_index_dict[node] = index

    # retrieve the weight of the graph, set weight to 1 if it is an unweighted graph
    try:
        for e in graph.edges(data=True):
            i, j, data = e
            index_i = node_index_dict[i]
            index_j = node_index_dict[j]
            distance[index_i][index_j] = data.get('weight', 1.0)
            # print("weight", distance[index_i][index_j])
    except:
        for e in graph.edges(data=True):
            i, j, data = e
            index_i = node_index_dict[i]
            index_j = node_index_dict[j]
            distance[index_i][index_j] = 1
            # print("weight", distance[index_i][index_j])

    # commmpute the shortest distance
    for k in range(nr_vertices):
        for i in range(nr_vertices):
            for j in range(nr_vertices):
                if distance[i][j] > distance[i][k] + distance[k][j]:
                    distance[i][j] = distance[i][k] + distance[k][j]

    return distance

# random similarity measure, might look for a better one later one
def similarity(node_i, node_j, distance_matrix):
    return 1 / (1 + distance_matrix[node_i][node_j])

def convert_to_similarity_matrix(distance_matrix):
    distance_len = len(distance_matrix)
    sim_matrix = np.zeros((distance_len, distance_len))
    for i in range(distance_len):
        for j in range(distance_len):
            if distance_matrix[i][j] != np.inf:
                sim = similarity(i, j, distance_matrix)
                sim_matrix[i][j] = sim
    return sim_matrix

def get_tsne_coordinates():
    return

def get_isomap_coordinates():
    return


distance_matrix = floyd_warshall_matrix(G)
print("distance matrix", distance_matrix)
similarity_matrix = convert_to_similarity_matrix(distance_matrix)
print("similarity matrix", similarity_matrix)



# distance_matrix = floyd_warshall_dict(G)
# print("distance matrix", distance_matrix)
