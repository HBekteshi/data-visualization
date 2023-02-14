import networkx
import pydot
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


def create_coordinates(width, height, adjancency_dict):
    coordinates = {}
    adjacencies = adjacency_dict 

    #if dictionary is empty return the coordinates
    if(not adjacencies):
        return coordinates

    for id, adj_nodes in adjacencies: #create dictionary with the size of the number of edges per node
        adjacencies.update(id=len(adj_nodes))

    max_adjancency = max(adjacencies.values()) #retrieve the max adjacency
    max_adjacencies = [n for n,v in adjacencies.items() if v == max_adjancency] #make list of all nodes with the max adjacency

    if(len(max_adajacencies) == 1): #still have to figure out how to do this part exactly with determining rings/coordinates etc.
        ...
    else:
        ...

    #generate dictionary without the keys from the previous maximum adjacencies
    for id in max_adjacencies:
        adjacencies.pop(key)
    
    create_coordinates(width, heigh, adjacencies) #recurse until empty dictionary