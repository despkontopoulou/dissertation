from heapq import heappop, heappush
import math

class Graph():
    def __init__(self, graph, coords):
        self.graph = graph
        self.coords = coords

    def get_neighbours(self, node):
        return self.graph.get(node,{})
    
    def get_coords(self,node):
        return self.coords.get(node)
    
class Astar():
    def __init__(self, graph):
        self.graph= graph
    
    def heuristic(self, node, goal):
        current_x, current_y = self.graph.get_coords(node)
        goal_x, goal_y = self.graph.get_coords(goal)
        return math.sqrt((goal_x-current_x)**2 +(goal_y-current_y)**2) # euclidean 

    def astar_search(self, start, goal):
        open = []
        heappush(open, (0,start))
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start,goal)}
        predecessors = {}

        while open:
            current_f, current= heappop(open)
            if current == goal:
                return self.reconstruct_path(predecessors, start, goal)
            
            for neighbour, weight in self.graph.get_neighbours(current).items():
                g_cost = g_costs[current] + weight

                if neighbour not in g_costs or g_cost < g_costs[neighbour]:
                    g_costs[neighbour] = g_cost
                    f_cost = g_cost + self.heuristic(neighbour, goal)
                    f_costs[neighbour]= f_cost
                    predecessors[neighbour] = current
                    heappush(open,(f_cost,neighbour))
        return []

    def reconstruct_path(self, predecessors, start, goal):
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = predecessors[current]
        path.append(start)
        return path[::-1]

graph = {
   "A": {"B": 3, "C": 3},
   "B": {"A": 3, "D": 3.5, "E": 2.8},
   "C": {"A": 3, "E": 2.8, "F": 3.5},
   "D": {"B": 3.5, "E": 3.1, "G": 10},
   "E": {"B": 2.8, "C": 2.8, "D": 3.1, "G": 7},
   "F": {"G": 2.5, "C": 3.5},
   "G": {"F": 2.5, "E": 7, "D": 10},
}

coords = {
   "A": (0,0),
   "B": (1,2),
   "C": (2,0),
   "D": (3,4),
   "E": (2,3),
   "F": (4,1),
   "G": (5,5)
}
graph_obj = Graph(graph, coords)
astar = Astar(graph_obj)

start = "A"
goal = "G"
path = astar.astar_search(start, goal)
print("Path:", path)
