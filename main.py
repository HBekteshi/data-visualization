import networkx
import pydot
import numpy as np
import math
G = networkx.Graph(networkx.nx_pydot.read_dot('data/LesMiserables.dot'))
#G = networkx.Graph(networkx.nx_pydot.read_dot('data/JazzNetwork.dot'))
#G = networkx.Graph(networkx.nx_pydot.read_dot('data/rome.dot'))



printing_mode = False

print("Data loaded")

adjacency_dict = {}
adjacencies = {}

for n in G.nodes():
    if printing_mode:
        print(f"Added node with id {n}")
    adjacency_dict[n] = []

# the les miserables import has a node with id '\\n' and no connections

    
# the first time an edge appears it is marked True, and it will be rendered
# the second time the same edge appears it is marked False, so it will not be rendered
    
weightcheck = True
weighted = True
for e in G.edges():
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


#print(G.edges('0'))
if printing_mode:
    print("adjacency_dict:",adjacency_dict)

max_edges = 0
most_connected_node_id = None

for id, adj_nodes in list(adjacency_dict.items()): #create dictionary with the size of the number of edges per node
    adjacencies[id] = len(adj_nodes)
    if len(adj_nodes) > max_edges:
        max_edges = len(adj_nodes)
        most_connected_node_id = id

def create_random_coordinates(width, height, adjacency_dict):
    coordinates = {}
    for n in adjacency_dict.keys():
        x_val = np.random.uniform(-width/2, width/2)
        y_val = np.random.uniform(-height/2, height/2)
        coordinates[n] = (x_val, y_val)
    return coordinates
    

#random_coordinates = create_random_coordinates(100, 100)

def create_solar_coordinates(width, height, adjacency_dict, deterministic = False):
    coordinates = {}

    max_adjacency = max(adjacencies.values()) #retrieve the max adjacency
    nr_rings = len(set(adjacencies.values())) #calculate # of rings based on the # of unique values in adjacency numbers

    rings_dict = assign_to_rings(adjacencies)

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


def calc_radius(height, width, i, nr_rings):
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
        radius = calc_radius(height, width, i, nr_rings)
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
        radius = calc_radius(height, width, i, nr_rings)
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

def create_radial_coordinates(width, height, node_list):
    coordinates = {}
    annulus_wedge_angles = {} #store each annulus_wedge_angle in here by node_id
    distance_between_layers = 50 #play with this number, this is for the distance between the C layers, later make it into a function maybe
    start_radius = 100 # initialize the radius of the first layer
    root_node = node_list[0][1]

    # base case:
    # if the list consists of one node, we put it in the middle of the screen and done
    if(len(node_list) == 1):
        coordinates[root_node] = (0,0)
        return coordinates
    
    #Initialize the root node
    coordinates[root_node] = (0,0)

    # conquer case:
    # first layer after the root
    # put the child in the middle of the annulus wedge
    direct_root_children = calc_direct_children(node_list, root_node)
    print(direct_root_children)
    for child_id in direct_root_children:
        child_angle = calc_ann_wedge(node_list, root_node, child_id, start_radius, start_radius + distance_between_layers)
        annulus_wedge_angles[child_id] = child_angle
        parent_x = coordinates[root_node][0]
        parent_y = coordinates[root_node][1]
        coordinates[child_id] = polar_to_cartesian(child_angle, start_radius, parent_x, parent_y)

    #
    # for other layers
    # use function in the slides for calculating the angle of each child

    # divide case:
    # recursively apply the conquer algorithm to the left and right side of the subtree until there are no children anymore

    return coordinates

def calc_ann_wedge(node_list, parent_id, child_id, radius1, radius2):
    length_child = len(calc_all_children(node_list, child_id))
    length_parent = len(calc_all_children(node_list, parent_id))
    radius_angle = radius1 / radius2
    acos_arg = length_child / (length_parent - 1)
    print(acos_arg)
    length_angle = 2 * math.acos(length_child / (length_parent - 1))
    wedge_angle = min(radius_angle, length_angle)

    return wedge_angle

def calc_all_children(node_list, parent_id):
    all_children = [] #including grandchildren, thus the whole subtree rooted at the parent_id

    for tuple in node_list:
        if tuple[0] in all_children:
            all_children.append(tuple[1])
            print("add child", tuple[1], "with parent", tuple[0], "to all_children list")
        elif tuple[0] == parent_id and tuple[0] != tuple[1]:
            all_children.append(tuple[1])
            print("add child", tuple[1], "with parent", tuple[0], "to all_children list")
        #else do nothing 
    return all_children

def calc_direct_children(node_list, parent_id):
    direct_children = []
    for tuple in node_list:
        if tuple[0] == parent_id and tuple[0] != tuple[1]:
            direct_children.append(tuple[1])
            print("add child", tuple[1], "with parent", tuple[0], "to direct_children list")

    return direct_children

def calc_radial_coordinates_children(node_list, parent_node, annulus_wedge_angles, coordinates, start_radius, radius_distance):
    direct_root_children = calc_direct_children(node_list, parent_node)
    for child in direct_root_children:
        child_id = child[1]
        child_angle = calc_ann_wedge(node_list, parent_node, child_id, start_radius, start_radius + radius_distance)
        annulus_wedge_angles[child_id] = child_angle
        parent_x = coordinates[parent_node][0]
        parent_y = coordinates[parent_node][1]
        coordinates[child_id] = polar_to_cartesian(start_radius, parent_x, parent_y)

node_list_dfs = [('11','11'), ('11','2'), ('2','1'), ('2','3'), ('3','4'), ('2','5'), ('2','6'), ('2','7'), ('2','8'), ('2','9'),
                 ('2','10'), ('11','12'), ('11', '13'), ('13', '24'), ('24', '17'), ('17', '18'), ('18', '27'), ('27', '25'), ('25', '26'),
                 ('26', '28'), ('28', '29'), ('29', '45'), ('29', '46'), ('28', '30'), ('30', '35'), ('35', '36'), ('36', '37')]

coordinates = create_radial_coordinates(500,500,node_list_dfs)