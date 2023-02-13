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