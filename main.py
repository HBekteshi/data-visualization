import networkx
import pydot
import numpy as np
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
    for n in G.nodes():
        x_val = np.random.uniform(-width/2, width/2)
        y_val = np.random.uniform(-height/2, height/2)
        coordinates[n] = (x_val, y_val)
    return coordinates

random_coordinates = create_random_coordinates(100, 100)

def create_coordinates(width, height, adjancency_dict):
    coordinates = {}
    adjacencies = adjacency_dict 

    for id, adj_nodes in list(adjacencies.items()): #create dictionary with the size of the number of edges per node
        adjacencies[id] = len(adj_nodes)

    max_adjancency = max(adjacencies.values()) #retrieve the max adjacency
    max_adjacencies = [n for n,v in adjacencies.items() if v == max_adjancency] #make a list of all the nodes with the max adjacency
    rings_dict = assign_to_rings(adjacencies, max_adjancency)

    if(len(max_adjacencies) == 1): #still have to figure out how to do this part exactly with determining rings/coordinates etc.
        ...
        #calculate centre coordinates here for max adjacency nodes
        #add coordinates to dictionary
    else:
        ...   
        #make first ring here
        #for loop to assign coordinates on the ring? --> random assignment for each node?

    for i in range(0,max_adjancency):
        max_adjacencies = [n for n,v in adjacencies.items() if v == i] #make list of all nodes with the current adjacency
        
        if(len(max_adjacencies) == 1):
            #give random place to this node
            ...
        else:
            #add to a ring?
            #how to determine coordinates of the ring?
            ...

    return coordinates

def assign_to_rings(adjacencies, max_adjancency):

    #create dictionary with rings, based on the # of rings
    nr_rings = len(set(adjacencies.values())) #calculate # of rings based on the # of unique values in adjacency numbers
    rings_dict = {}
    for ring in set(adjacencies.values()): #create a ring for each unique value in the adjacencies, with that key value as the key
        rings_dict[ring] = []

    #for every node, add it to the ring with the corresponding size id
    for node, size in adjacencies.items(): 
        rings_dict[size].append(node)

    return rings_dict


def calc_radius(height, width, size):
    #calculate the radius of a ring based on the height, width and number of nodes on the ring 
    return

#coordinates = create_coordinates(100,100, adjacency_dict)