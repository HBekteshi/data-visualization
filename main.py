import networkx
import pydot
import numpy as np
import math
G = networkx.Graph(networkx.nx_pydot.read_dot('data/LesMiserables.dot'))

print("Data loaded")

adjacency_dict = {}

for n in G.nodes():
    print(f"Added node with id {n}")
    adjacency_dict[n] = []

# the les miserables import has a node with id '\\n' and no connections

    
# the first time an edge appears it is marked True, and it will be rendered
# the second time the same edge appears it is marked False, so it will not be rendered
    
for e in G.edges():
    u, v = e
    adjacency_dict[u].append((v,True))
    adjacency_dict[v].append((u,False))
    print(f"Added edge from {u} to {v}")


#print(G.edges('0'))
print(adjacency_dict)

def create_random_coordinates(width, height):
    coordinates = {}
    for n in adjacency_dict.keys():
        x_val = np.random.uniform(-width/2, width/2)
        y_val = np.random.uniform(-height/2, height/2)
        coordinates[n] = (x_val, y_val)
    return coordinates

random_coordinates = create_random_coordinates(100, 100)

def create_solar_coordinates(width, height, adjacency_dict):
    coordinates = {}
    adjacencies = {}

    for id, adj_nodes in list(adjacency_dict.items()): #create dictionary with the size of the number of edges per node
        adjacencies[id] = len(adj_nodes)

    max_adjacency = max(adjacencies.values()) #retrieve the max adjacency
    max_adjacencies = [n for n,v in adjacencies.items() if v == max_adjacency] #make a list of all the nodes with the max adjacency
    nr_rings = len(set(adjacencies.values())) #calculate # of rings based on the # of unique values in adjacency numbers
    rings_dict = assign_to_rings(adjacencies, max_adjacency)
    coordinates = convert_to_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width)

    print("Which nodes belong to which ring")
    print(rings_dict)

    print("the coordinates of each node")
    print(coordinates)

    return coordinates

def assign_to_rings(adjacencies, max_adjacency):

    #create dictionary with rings, based on the # of rings
    rings_dict = {}
    for ring in set(adjacencies.values()): #create a ring for each unique value in the adjacencies, with that key value as the key
        rings_dict[ring] = []

    #for every node, add it to the ring with the corresponding size id
    for node, size in adjacencies.items(): 
        rings_dict[size].append(node)

    return rings_dict


def calc_radius(height, width, size, nr_rings):
    #calculate the radius of a ring based on the height, width, the # of nodes on the ring and the # of rings 
    #TODO see if we can come up with a better function for this, this is just a test
    if(height<width):
        radius = (height/2) / nr_rings
    else:
        radius = (width/2) / nr_rings
    return radius

def polar_to_cartesian(angle, radius, center_x, center_y):
    x_val = center_x + radius * math.cos(angle)
    y_val = center_y + radius * math.sin(angle)
    return (x_val, y_val)   


def convert_to_solar_coordinates(rings_dict, nr_rings, max_adjacency, height, width):
    coordinates = {}

    # if the ring with the highest adjacency has just one node, put this one in the middle of the screen.
    # otherwise calculate the radius of the ring and coordinates of the nodes
    if(len(rings_dict[max_adjacency]) == 1):
        node_id = rings_dict[max_adjacency][0]
        coordinates[node_id] = (0,0)
    else:
        radius = calc_radius(height, width, len(rings_dict[max_adjacency]), nr_rings)
        for node_id in rings_dict[max_adjacency]:
            angle = np.random.uniform(0,360) #generate random angle
            coordinates[node_id] = polar_to_cartesian(angle, radius,0,0)
    
    #after the highest ring, ring centers get random coordinates
    #generate random angle for each node --> convert polar coordinates (radius, angle) to cartesian coordinates
    for ring, nodelist in rings_dict.items():
        radius = calc_radius(height, width, len(nodelist), nr_rings)
        x_center = np.random.uniform(-width/2, width/2)
        y_center = np.random.uniform(-height/2, height/2)
        for node_id in nodelist:
            angle = np.random.uniform(0,360) #generate random angle
            coordinates[node_id] = polar_to_cartesian(angle, radius,x_center,y_center)

    return coordinates

coordinates = create_solar_coordinates(100,100, adjacency_dict)