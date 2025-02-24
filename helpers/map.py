import osmnx as ox

def load_place(place_name):
    G = ox.graph_from_place(place_name, network_type='drive')
    G= ox.add_edge_speeds(G)
    G= ox.add_edge_travel_times(G)
    nodes, edges= ox.graph_to_gdfs(G)
    geojson=edges.to_json()
    return G, geojson

def convert_to_graph(G):
    graph_data={}
    coords={}

    for node, data in G.nodes(data=True):
        coords[node]=(data['x'], data['y'])

    for u,v, data in G.edges(data=True):
        travel_time=data.get("travel_time",1)
        if u not in graph_data:
            graph_data[u]={}
        graph_data[u][v]=travel_time

        if v not in graph_data:
            graph_data[v]={}
        graph_data[v][u]=travel_time
    return graph_data, coords



