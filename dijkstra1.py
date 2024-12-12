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


def dijkstra(G, start, end):
    # initialize distances: set all nodes to infinity since the distance is uknown
    distances= {node: float("inf") for node in G.nodes}
    distances[start] = 0 # distance from start node to self is 0

    # priority queue initialization
    pq = [(0,start)]
    heapify(pq) #make list a heap

    # set to hold visited nodes
    visited = set()

    # predecessors dictionary to reconstruct path
    predecessors = {node: None for node in G.nodes}

    # while the priority queue is not empty
    while pq:
        # pop node with the smallest distance
        current_distance, current_node = heappop(pq) 

        # skip if current node has been visited
        if current_node in visited:
            continue 
        visited.add(current_node) # else add to visited

        # stop early if we reach the end node
        if current_node == end:
            break

        # explore neighbors of current node
        for neighbor in G.neighbors(current_node):
            # get the weight of the edge and from current node to neighbor and calulate potential distance
            edge_weight = G[current_node][neighbor].get('weight',1)
            distance = current_distance + edge_weight # -potential- distance

            if distance < distances[neighbor]: # if the distance is shorter, update it
                distances[neighbor] = distance
                predecessors[neighbor] = current_node # record that the path to neighbor is via the current_node 
                heappush(pq,(distance, neighbor)) # add neighbor to priority queue with new distance

    # return shortest distance to end node and the reconstructed path
    return distances[end], reconstruct_path(predecessors, start, end)

def reconstruct_path(predecessors, start, end):
    path = []
    current = end # traceback from destination node
    # follow predecessors until start node
    while current is not None:
        path.insert(0, current) # insert current node at the beginning
        current = predecessors[current] # move to the predecessor of the current node
    return path if path[0] == start else [] #the path is valid if the first node is the start node

# test
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
plt.show()

distance, path = dijkstra(G,'A','G')
print(f"Shortest distance from A to G: {distance}")
print(f"Path: {path}")