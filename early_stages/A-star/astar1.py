from heapq import heappop, heappush
class Node():
    def __init__(self, x, y, g_cost=0, h_cost=0): # node class init
        self.x = x # coords
        self.y=y
        self.g_cost = g_cost # cost of path so far 
        self.h_cost = h_cost # heuristic, estimated cost from current to end
        self.f_cost = self.g_cost + self.h_cost #total cost estimate
        self.parent= None #parent

    def __lt__(self, other):
        return self.f_cost< other.f_cost

class Astar:
    def __init__(self,grid):
        self.open = [] # list initializations
        self.closed = []
        self.grid = grid # grid initialization
        self.rows = len(grid)
        self.cols = len(grid[0])

    def is_valid(self, x, y): # check if node is in grid and not an obstacle (obstacles in grid are grid[x][y]==1)
        return(0 <= x < self.rows) and (0 <= y < self.cols) and (self.grid[x][y] == 0)
            

    def get_neighbours(self, node):
        directions = [[1,0],[0,1],[-1,0],[0,-1]] # possible directions for grid
        neighbours = []
        
        for dx, dy in directions: #calculate neighbour coords
            nx = node.x + dx
            ny= node.y + dy
            if (self.is_valid(nx,ny)): # check if valid
                neighbours.append(Node(nx,ny)) #add to list if neighbours of node
        return neighbours
    
    def heuristic(self, node, goal):# manhattan distance
        return abs(node.x - goal.x) + abs(node.y - goal.y) # |x1-x2|+|y1-y2|
        
    def reconstruct_path(self, end_node):
        path=[] # init path list
        current = end_node #backtracking, start from last node
        while current: #while we have a node to check
            path.append((current.x, current.y)) # add current to path
            current=current.parent # check currents parents
        return path[::-1] # inverse

    def search(self,start_t,goal_t): # main loop
        start= Node(start_t[0],start_t[1]) #init nodes using coords
        goal = Node(goal_t[0], goal_t[1])

        start.h_cost= self.heuristic(start, goal) # calc first heuristic
        start.f_cost= start.h_cost # first cost is just the heuristic

        open= []
        heappush(open, start) # put start in min priority queue
        closed= set()  # init closed set
            
        while open: #while there are nodes we can check
            current= heappop(open) # get the priority node

            if (current.x , current.y )in closed: # if already checked pass
                continue

            closed.add((current.x,current.y)) #else add to closed 

            if current.x == goal.x and current.y == goal.y: #if its the goal node end and reconstruct path
                return self.reconstruct_path(current)
            
            neighbours= self.get_neighbours(current) #else get neighbours
            for neighbour in neighbours:
                if (neighbour.x, neighbour.y) in closed:#check them if not in closed
                    continue
                neighbour.g_cost= current.g_cost + 1 # calc costs (1 more to move)
                neighbour.h_cost= self.heuristic(neighbour, goal)
                neighbour.f_cost= neighbour.g_cost + neighbour.h_cost
                neighbour.parent = current# set current as neighbours parent
                heappush(open, neighbour) #add neighbour to list to check
               
        return []

#test
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0],
]

astar = Astar(grid)
start = (0, 0)  
goal = (4, 4)   
path = astar.search(start, goal)
print("Path:", path)