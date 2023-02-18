import networkx
import pydot
import numpy as np
import math
G = networkx.Graph(networkx.nx_pydot.read_dot('data/LesMiserables.dot'))

printing_mode = False

print("Data loaded")

adjacency_dict = {}

for n in G.nodes():
    if printing_mode:
        print(f"Added node with id {n}")
    adjacency_dict[n] = []

# the les miserables import has a node with id '\\n' and no connections

    
# the first time an edge appears it is marked True, and it will be rendered
# the second time the same edge appears it is marked False, so it will not be rendered
    
for e in G.edges():
    u, v = e
    adjacency_dict[u].append((v,True))          # for directed graph, make sure direction is u-->v for True 
    adjacency_dict[v].append((u,False))
    if printing_mode:
        print(f"Added edge from {u} to {v}")


#print(G.edges('0'))
if printing_mode:
    print("adjacency_dict:",adjacency_dict)

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
    adjacencies = {}

    for id, adj_nodes in list(adjacency_dict.items()): #create dictionary with the size of the number of edges per node
        adjacencies[id] = len(adj_nodes)

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
