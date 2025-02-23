class Graph:
    def __init__(self, graph, coords=None):
        self.graph = graph #info of nodes and weights
        self.coords = coords #coords of each node for spacial info for heuristic of a*

    def get_neighbours(self, node):
        return self.graph.get(node, {}) #gets neighbours of node

    def get_coords(self, node):
        if self.coords:
            return self.coords.get(node, (0, 0))
        return 0,0

    def reconstruct_path(self, predecessors, start, end):
        path = []
        current = end
        while current is not None:
            path.insert(0, current)
            current = predecessors.get(current)
        if not path or path[0] != start:
            return []
        return path