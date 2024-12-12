import time
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


def dijkstra(graph, start, end):
    # initialize distances: set all nodes to infinity since the distance is unknown
    distances = {node: float("inf") for node in graph.nodes}
    distances[start] = 0 # distance from start node to self is 0

    # priority queue initialization
    pq = [(0,start)]
    heapify(pq) #make list a heap

    # set to hold visited nodes
    visited = set()

    # predecessors dictionary to reconstruct path
    predecessors = {node: None for node in graph.nodes}

    # while the priority queue is not empty
    while pq:
        # pop node with the smallest distance, set to current
        current_distance, current_node = heappop(pq) 

        # skip if current node has been visited
        if current_node in visited:
            continue 
        visited.add(current_node) # else add to visited

        # stop early if we reach the end node
        if current_node == end:
            break

        # explore neighbors of current node
        for neighbor in graph.neighbors(current_node):
            # get the weight of the edge and from current node to neighbor and calculate potential distance
            edge_weight = graph[current_node][neighbor].get('weight',1) # if graph were unweighted, 1 would be the default weight 
            distance = current_distance + edge_weight # -potential- distance

            # if the distance is shorter, update it
            if distance < distances[neighbor]: 
                distances[neighbor] = distance
                predecessors[neighbor] = current_node # record that the path to neighbor is via the current_node 
                heappush(pq,(distance, neighbor)) # add neighbor to priority queue with new distance

    # return shortest distance to end node and the reconstructed path
    if distances[end] == float("inf"):
        print(f"No path found")
        return float("inf"), []
    else:
        return distances[end], reconstruct_path(predecessors, start, end)

def reconstruct_path(predecessors, start, end):
    path = []
    current = end # traceback from destination node
    # follow predecessors until start node
    while current is not None:
        path.insert(0, current) # insert current node at the beginning
        current = predecessors[current] # move to the predecessor of the current node
    
    if not path or path[0] != start:
        return []
    return path 


# # test1
# # undirected weighted graph with path from start to end
# G = nx.Graph()
# G.add_edge('A','B', weight = 3)
# G.add_edge('A','C', weight = 3)
# G.add_edge('B','D', weight = 3.5)
# G.add_edge('B','E', weight = 2.8)
# G.add_edge('C','E', weight = 2.8)
# G.add_edge('C','F', weight = 3.5)
# G.add_edge('D','E', weight = 3.2)
# G.add_edge('D','G', weight = 10)
# G.add_edge('E','G', weight = 7)
# G.add_edge('F','G', weight = 2.5)

# edge_labels= nx.get_edge_attributes(G,'weight')
# pos=nx.planar_layout(G)
# nx.draw(G,pos, with_labels = True)
# nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
# plt.title("Weighted graph with path to all nodes")
# plt.show()

# distance, path = dijkstra(G,'A','G')
# print(f"Shortest distance from A to G: {distance}")
# print(f"Path: {path}\n")


# # test2
# # undirected weighted graph without path from start to end
# H = nx.Graph()
# H.add_edge('A','B', weight = 3)
# H.add_edge('E','D', weight = 3)
# H.add_edge('D','F', weight = 3.5)

# edge_labels= nx.get_edge_attributes(H,'weight')
# pos=nx.planar_layout(H)
# nx.draw(H,pos, with_labels = True)
# nx.draw_networkx_edge_labels(H, pos, edge_labels=edge_labels)
# plt.title("Weighted graph with no path to all nodes")
# plt.show()

# distance, path = dijkstra(H,'A','F')
# print(f"Shortest distance from A to F: {distance}")
# print(f"Path: {path}\n")

# #test3
# # directed graph
# DG = nx.DiGraph()
# DG.add_edge('A', 'B', weight= 3.5)
# DG.add_edge('A', 'C', weight= 1.2)
# DG.add_edge('B', 'D', weight= 2)
# DG.add_edge('C', 'D', weight= 4.5)
# DG.add_edge('C', 'E', weight= 2)
# DG.add_edge('D', 'E', weight= 3)

# edge_labels = nx.get_edge_attributes(DG, 'weight')
# pos = nx.spring_layout(DG)
# nx.draw(DG, pos, with_labels=True, arrows=True)
# nx.draw_networkx_edge_labels(DG, pos, edge_labels=edge_labels)
# plt.title("Directed Weighted Graph")
# plt.show()

# distance, path = dijkstra(DG,'A','E')
# print(f"Shortest distance from A to E: {distance}")
# print(f"Path: {path}\n")