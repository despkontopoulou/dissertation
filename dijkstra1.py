import networkx as nx
import matplotlib.pyplot as plt
from heapq import heapify, heappop, heappush
# Dijkstras algorithm in python
# initialize graph and node distances
    # set root node
    # set distances between b and all other nodes as inf 
    # set distance from root to self to 0
# Iteratively
    # choose node with smallest value (distance) and set it as "current"
    # visit all of current's adjacent nodes (adj)
    # update adj's distance to current's distance + weight to adj (if it is smaller than existing value)

    # after all adj are visited we mark current as visited. Its value is already the shortest path from source to current
    # then go back to step 1 and choose the unexplored node with the smallest value


G = nx.Graph()
G.add_edge('A','B', weight = 3)
G.add_edge('A','C', weight = 3)
G.add_edge('B','D', weight = 3.5)
G.add_edge('B','E', weight = 2.8)
G.add_edge('C','E', weight = 2.8)
G.add_edge('C','F', weight = 3.5)
G.add_edge('D','E', weight = 3.2)
G.add_edge('D','G', weight = 10)
G.add_edge('E','G', weight = 7)
G.add_edge('F','G', weight = 2.5)

edge_labels= nx.get_edge_attributes(G,'weight')
pos=nx.planar_layout(G)
nx.draw(G,pos, with_labels = True)
plt.title("weighted graph")
#plt.show()


def dijkstra(G, start):
    distances= {node: float("inf") for node in G.nodes}
    distances[start] = 0

    #priority queue initialization
    pq = [(0,start)]
    heapify(pq)
    #set to hold visited nodes
    visited = set()

    while pq:
        current_distance, current_node = heappop(pq) #get node with min distance

        if current_node in visited:
            continue #skip if current node has been visited
        visited.add(current_node) # else add to visited

        for neighbor in G.neighbors(current_node):
            edge_weight = G[current_node][neighbor].get('weight',1)
            distance = current_distance + edge_weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heappush(pq,(distance, neighbor))

    return distances
    
print("Distances from A")
distances_from_a= dijkstra(G, 'A')
print(distances_from_a, "\n")
to_F= distances_from_a['F']
print(f"The shortest distance from A to F is {to_F}\n")

print("Distances from B")
distances_from_b= dijkstra(G, 'B')
print(distances_from_b, "\n")
to_F= distances_from_b['F']
print(f"The shortest distance from B to F is {to_F} \n")