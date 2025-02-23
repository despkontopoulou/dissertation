from heapq import heappop, heappush
import math

class Astar:
    def __init__(self, graph_obj):
        self.graph_obj = graph_obj #get nodes, edges and weights from graph object

    def heuristic(self, node, goal):  # get euclidean distance
        current_x, current_y = self.graph_obj.get_coords(node)
        goal_x, goal_y = self.graph_obj.get_coords(goal)
        return math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    def astar_search(self, start, goal):  # main loop
        heap = []  # min priority heap
        # we dont have a closed list, we use the g_costs list to track distances
        heappush(heap, (0, start))  # push start node to open
        g_costs = {start: 0}  # g_costs storing from the start (equivalent to distances list of dijkstra)
        f_costs = {start: self.heuristic(start, goal)}  # f cost list, uses both the sctual distance and the heuristic
        predecessors = {}  # stores path (has the predecessors of all nodes)

        while heap:
            current_f, current = heappop(heap)  # pop node with smallest f cost
            if current == goal:  # if we reach the goal end loop and reconstruct path
                return self.graph_obj.reconstruct_path(predecessors, start, goal)

            for neighbour, weight in self.graph_obj.get_neighbours(
                    current).items():  # explore all neighbours of current
                g_cost = g_costs[current] + weight  # calculate g cost from start to neighbour

                if neighbour not in g_costs or g_cost < g_costs[
                    neighbour]:  # if neighbour has not been visited or there is a bigger g cost
                    g_costs[neighbour] = g_cost  # update g
                    f_cost = g_cost + self.heuristic(neighbour, goal)  # calculate f
                    f_costs[neighbour] = f_cost  # update f
                    predecessors[neighbour] = current  # save predecessor
                    heappush(heap, (f_cost, neighbour))  # push neighbour into heap with new cost
        return []