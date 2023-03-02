import networkx
import pydot
import numpy as np
import math
import copy


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

def create_force_layout_coordinates(vertex_object_dict, width, height, initial_coords, C = 1, max_iterations = 100):
    delta = 0.01 # given number within the range (0,1]`
    iteration_count = 0
    coords_dict = initial_coords.copy()
    area = width * height
    
    nr_vertices = len(initial_coords.keys())

    while iteration_count < max_iterations:
        coords_dict = force_iteration(vertex_object_dict, coords_dict, delta,area, nr_vertices, C)
        iteration_count += 1

    return coords_dict


def force_iteration(vertex_object_dict, old_coordinates_dict, delta_value, area, nr_vertices, C):
    new_coordinates_dict = copy.deepcopy(old_coordinates_dict)

    for id in (old_coordinates_dict.keys()):
        old_coords_tuple = old_coordinates_dict[id]             # (x,y) tuple
        force = calc_sum_force(vertex_object_dict, id, old_coords_tuple, old_coordinates_dict, area, nr_vertices, C)
        new_coordinates_dict[id] = (old_coords_tuple[0] + delta_value * force[0], old_coords_tuple[1] + delta_value * force[1])
       # print("node",id,"gets a force push of",force)

    return new_coordinates_dict

def calc_sum_force(vertex_object_dict, current_id, old_coords_tuple, old_coordinates_dict, area, nr_vertices, C):
    #adj_nodes = calc_direct_children() #to check again          # need node list and parent id  # this only works for a tree structure
    adj_nodes = []

    for edge, next in vertex_object_dict[current_id].edges:
        adj_nodes.append(next.id)

    force = [0,0]
    length = calc_ideal_length(area, nr_vertices, C)
   # print("ideal length of an edge is calculated to be", length)

    for a_node_id in adj_nodes:
        a_node_coords_tuple = old_coordinates_dict[a_node_id]
        dx, dy = calc_attr_force_eades(1, a_node_coords_tuple, old_coords_tuple) # change 1 to length for fruchterman and vice versa
        force[0] += dx
        force[1] += dy
    
  # print("with only attractive forces, node", current_id,"gets a force of",force)

    for node_id in old_coordinates_dict.keys():
        if node_id != current_id:
            node_coords = old_coordinates_dict[node_id]

            if node_coords != old_coords_tuple:             # check that the nodes are not in the same location
              #  print("inequality testing", node_coords,old_coords_tuple)
                dx, dy = calc_rep_force_eades(1, node_coords, old_coords_tuple) #change 1 to length for fruchterman and vice versa
                force[0] += dx
                force[1] += dy

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
    if eucl_dist == 0:
        print("distance zero:", x1, x2, y1, y2)
    return eucl_dist

def calc_unit_vec(x1, y1, x2, y2):
    """ 
    input: x1, y1, x2, y2 coordinates that represent node 1 and node 2 repestively
    output: a tuple (x,y) for the unit vector""" 
    magnitude = calc_eucl_dist(x1, y1, x2, y2)
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
    rep_forcex = (length / (eucl_dist * eucl_dist)) * unit_vec12[0]
    rep_forcey = (length / (eucl_dist * eucl_dist)) * unit_vec12[1]
    
    
    return (rep_forcex, rep_forcey)

def calc_attr_force(length, node1, node2):
    """
    input: length: ideal length of an edge, float, node1 and node2: a tuple of (x,y) coordinates
    output: a float tuple (x,y) that indicates the attractive force""" 
    x1 = node1[0]
    y1 = node1[1]
    x2 = node2[0]
    y2 = node2[1]
    unit_vec12 = calc_unit_vec(x1, y1, x2, y2)
    eucl_dist = calc_eucl_dist(x1, y1, x2, y2)
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
    attr_forcex = 2 * math.log((eucl_dist / length)) * unit_vec12[0]
    attr_forcey = 2 * math.log((eucl_dist / length)) * unit_vec12[1]

    return (attr_forcex, attr_forcey)


#create_radial_coordinates(500,500,node_list_dfs)
#print(coordinates)pipenv run python application.py
length = calc_ideal_length(50, 50, 2)
print("length", length)
eucl = calc_eucl_dist(5, 8 , 10 ,13)
print("euclidean = ", eucl) 
unit = calc_unit_vec(5, 8, 10, 13)
print("unit vec = ", unit)
rep_force = calc_rep_force_eades(5, (5,8), (10,13))
attr_force = calc_attr_force_eades(5, (5,8), (10,13))
print("rep =", rep_force)
print("attr =", attr_force)