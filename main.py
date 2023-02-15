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

for e in G.edges():
    u, v = e
    adjacency_dict[u].append(v)
    adjacency_dict[v].append(u)
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
print(random_coordinates)

def create_coordinates(width, height, adjancency_dict):
    coordinates = {}
    adjacencies = adjacency_dict 

    for id, adj_nodes in adjacencies: #create dictionary with the size of the number of edges per node
        adjacencies.update(id=len(adj_nodes))

    max_adjancency = max(adjacencies.values()) #retrieve the max adjacency
    nr_rings = len(set(adjacencies.values())) #calculate # of rings based on the # of unique values

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

def calc_radius(height, width, size):
    #calculate the radius of a ring based on the height, width and number of nodes on the ring 
    return