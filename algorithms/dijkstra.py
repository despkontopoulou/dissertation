from heapq import heapify, heappop, heappush

# dijkstra implementation with custom node class
class Dijkstra:
    def __init__(self, graph_obj):
        self.graph_obj = graph_obj

    def dijkstra_search(self, start, end):
        distances = {node: float("inf") for node in self.graph_obj.graph}
        distances[start] = 0
        pq = [(0, start)]
        heapify(pq)
        visited = set()
        predecessors = {node: None for node in self.graph_obj.graph}

        while pq:
            current_distance, current_node = heappop(pq)

            if current_node in visited:
                continue
            visited.add(current_node)

            if current_node == end:
                break

            for neighbour, weight in self.graph_obj.get_neighbours(current_node).items():
                if neighbour not in distances:
                    print(f"ERROR: Neighbour {neighbour} not in distances!")
                    print(f"Current node: {current_node}")
                    print(f"Graph neighbors: {self.graph_obj.get_neighbours(current_node)}")
                    return float("inf"), []
                distance = current_distance + weight
                if distance < distances[neighbour]:
                    distances[neighbour] = distance
                    predecessors[neighbour] = current_node
                    heappush(pq, (distance, neighbour))

        if distances[end] == float("inf"):
            print("No path found")
            return float("inf"), []
        else:
            return self.graph_obj.reconstruct_path(predecessors, start, end)
