from heapq import heappop, heappush, heapify
class Node():
    def __init__(self, x, y, g_cost, h_cost):
        self.x=x
        self.y=y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = self.g_cost + self.h_cost
        self.parent= None #######?

    def compare(self, other):
        return self.f_cost< other.f_cost

class Astar:
    def __init__(self,graph):
        self.open = []
        self.closed = []
        self.graph = graph
############
    def get_neighbours(self, node):
        pass
    def heuristic(self, node, goal):
        pass
    def reconstruct_path(self, goal_node):
        pass
    def update(self, node, g_cost, h_cost):    
        pass
###########
    def search(self,start,goal):
        self.open.append(start)
        heapify(open)
        current=heappop(open)
        self.closed.append(current)

        if current==goal:
            return self.reconstruct_path(goal)###########
        neighbours= self.get_neighbours(current)

        for neighbour in neighbours:
            if neighbour in self.closed:
                continue
            g_cost= current.g_cost + 1 # 1 more to move
            h_cost= self.heuristic(neighbour, goal)
            f_cost= g_cost + h_cost

            if neighbour in self.open:
                if neighbour.f_cost> f.cost:
                    self.update(neighbour, g_cost, h_cost)######
            else:
                self.update(neighbour, g_cost, h_cost)

        return None

    
