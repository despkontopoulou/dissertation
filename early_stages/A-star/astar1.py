from heapq import heappop, heappush, heapify
class Node():
    def __init__(self, x, y, g_cost=0, h_cost=0):
        self.x=x
        self.y=y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = self.g_cost + self.h_cost
        self.parent= None #######?

    def __lt__(self, other):
        return self.f_cost< other.f_cost

class Astar:
    def __init__(self,grid):
        self.open = []
        self.closed = []
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def is_valid(self, x, y):
        return(0 <= x < self.rows) and (0 <= y < self.cols) and (self.grid[x][y] == 0)
            

    def get_neighbours(self, node):
        directions = [[1,0],[0,1],[-1,0],[0,-1]]
        neighbours = []
        
        for dx, dy in directions:
            nx = node.x + dx
            ny= node.y + dy
            if (self.is_valid(nx,ny)):
                neighbours.append(Node(nx,ny))
        return neighbours
    
    def heuristic(self, node, goal):# manhattan distance for now
        return abs(node.x - goal.x) + abs(node.y - goal.y)
        
    def reconstruct_path(self, end_node):
        path=[]
        current = end_node
        while current:
            path.append((current.x, current.y))
            current=current.parent
        return path[::-1]
  

###########
    def search(self,start_t,goal_t):
        start= Node(start_t[0],start_t[1])
        goal = Node(goal_t[0], goal_t[1])

        start.h_cost= self.heuristic(start, goal)
        start.f_cost= start.h_cost

        open= []
        heappush(open, start)
        closed= set() 
            
        while open:
            current= heappop(open)

            if (current.x , current.y )in closed:
                continue

            closed.add((current.x,current.y))

            if current.x == goal.x and current.y == goal.y:
                return self.reconstruct_path(current)
            
            neighbours= self.get_neighbours(current)
            for neighbour in neighbours:
                if (neighbour.x, neighbour.y) in closed:
                    continue
                neighbour.g_cost= current.g_cost + 1 # 1 more to move
                neighbour.h_cost= self.heuristic(neighbour, goal)
                neighbour.f_cost= neighbour.g_cost + neighbour.h_cost
                neighbour.parent = current
                heappush(open, neighbour)
               
        return []

        
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0],
]

astar = Astar(grid)
start = (0, 0)  # Top-left corner
goal = (4, 4)   # Bottom-right corner

path = astar.search(start, goal)
print("Path:", path)