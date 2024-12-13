from heapq import heapify, heappop, heappush
# dijkstra implementation with custom node class
class Graph():
    def __init__(self, graph):
        self.graph = graph

    def dijkstra(self, start, end):
        #create distances dictionary, initialize distances to inf and make min priority heap
        distances = {node: float("inf") for node in self.graph}
        distances[start] = 0
        pq = [(0, start)]
        heapify(pq)
        visited = set()
        predecessors = {node: None for node in self.graph}

        while pq:
            current_distance, current_node = heappop(pq)

            if current_node in visited:
                continue
            visited.add(current_node)

            if current_node == end:
                break
        
            for neighbour, weight in self.graph[current_node].items():
                distance = current_distance + weight
                if distance < distances[neighbour]:
                    distances[neighbour] = distance
                    predecessors[neighbour] = current_node
                    heappush(pq,(distance,neighbour))

        if distances[end] == float("inf"):
            print("No path found")
            return float("inf"),[]
        else:
            return distances[end], self.reconstruct_path(predecessors,start, end)
    
    def reconstruct_path(self, predecessors, start, end):
        path = []
        current = end
        while current is not None:
            path.insert(0,current)
            current = predecessors[current]
        if not path or path[0]!= start:
            return []
        return path
    

# graph class uses a predefined dictionary
graph = {
   "A": {"B": 3, "C": 3},
   "B": {"A": 3, "D": 3.5, "E": 2.8},
   "C": {"A": 3, "E": 2.8, "F": 3.5},
   "D": {"B": 3.5, "E": 3.1, "G": 10},
   "E": {"B": 2.8, "C": 2.8, "D": 3.1, "G": 7},
   "F": {"G": 2.5, "C": 3.5},
   "G": {"F": 2.5, "E": 7, "D": 10},
}

G = Graph(graph)
start = "A"
end = "G"
distance, path = G.dijkstra(start, end)
print(f"The distance from {start} to {end} is {distance} and the path is {path}")

start = "B"
end = "F"
distance, path = G.dijkstra(start, end)
print(f"The distance from {start} to {end} is {distance} and the path is {path}")


    