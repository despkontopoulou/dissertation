import osmnx as ox
#Creation of graph from map
def load_place(place_name):
    G = ox.graph_from_place(place_name, network_type='drive', simplify=False)
    G= ox.add_edge_speeds(G)
    G= ox.add_edge_travel_times(G)
    nodes, edges= ox.graph_to_gdfs(G)
    geojson=edges.to_json()
    return G, geojson

def convert_to_graph(G):
    graph_data={}
    coords={}

    for node, data in G.nodes(data=True):
        coords[node]=(data['x'], data['y']) #loop through nodes and save their coordinates
        graph_data[node]={}

    for u,v, data in G.edges(data=True): #loop through all edges
        travel_time=data.get("travel_time",1) #get precompiled travel time else default 1
        oneway=data.get("oneway") #check if the road is oneway else false

        graph_data[u][v]=travel_time #store traveltime

        if not oneway:
            graph_data[v][u]=travel_time

    return graph_data, coords



