import networkx
import pydot
G = networkx.Graph(networkx.nx_pydot.read_dot('data/LesMiserables.dot'))

for n in G.nodes():
    print(f"Found node with id {n}")
for e in G.edges():

    u, v = e
    print(f"Edge from {u} to {v}")
    print(G.edges('0')) # Handy functionality =)
    print(G.edges('1'))