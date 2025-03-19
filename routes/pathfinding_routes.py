import folium
import osmnx.graph
from flask import Blueprint, request, jsonify, render_template
import time
from algorithms.astar import Astar
from algorithms.dijkstra import Dijkstra
from helpers.graph import Graph
import osmnx as ox

from helpers.map import load_place, convert_to_graph

#graph blueprint
pathfinding_bp= Blueprint('pathfinding', __name__)

# #test graph
# graph_data={
#     "A": {"B": 3, "C": 3},
#     "B": {"A": 3, "D": 3.5, "E": 2.8},
#     "C": {"A": 3, "E": 2.8, "F": 3.5},
#     "D": {"B": 3.5, "E": 3.1, "G": 10},
#     "E": {"B": 2.8, "C": 2.8, "D": 3.1, "G": 7},
#     "F": {"G": 2.5, "C": 3.5},
#     "G": {"F": 2.5, "E": 7, "D": 10}
# }
#
# coords = {
#     "A": (0, 0), "B": (1, 2), "C": (2, 0),
#     "D": (3, 4), "E": (2, 3), "F": (4, 1), "G": (5, 5)
# }
place_name = "Manhattan, New York, USA"
G , geojson = load_place(place_name)
graph_data, coords = convert_to_graph(G)
graph_obj = Graph(graph_data,coords) # create graph
astar = Astar(graph_obj)
dijkstra = Dijkstra(graph_obj)


@pathfinding_bp.route('/get_nearest_node', methods=['GET'])
def get_nearest_node():
    lat = request.args.get('lat', type=float)
    lon = request.args.get('lon', type=float)

    if lat is None or lon is None:
        return jsonify({"error": "Latitude and longitude required"}), 400

    nearest_node = ox.distance.nearest_nodes(G, lon, lat)  # Get closest OSM node
    return jsonify({"node_id": nearest_node})

@pathfinding_bp.route('/find_path', methods=['GET']) # API
def find_path():
    start = request.args.get("start", type=int)
    goal = request.args.get("goal", type=int)

    if not start or not goal or start not in graph_data or goal not in graph_data:
        return jsonify({"error": "start or goal are invalid"}), 400

    #Dijkstra
    dijkstra_start = time.perf_counter() #start time of dijkstra
    dijkstra_path = dijkstra.dijkstra_search(start, goal) #execution of algorithm
    dijkstra_time = time.perf_counter() - dijkstra_start #total time of execution

    #A-star
    astar_start = time.perf_counter()
    astar_path = astar.astar_search(start, goal)
    astar_time = time.perf_counter() - astar_start

    return jsonify({
        "dijkstra": {"path":dijkstra_path, "time":dijkstra_time},
        "astar": {"path":astar_path, "time":astar_time}
    })

@pathfinding_bp.route('/visualize_path')
def visualize_path():
    start= request.args.get("start", type=int)
    goal= request.args.get("goal", type=int)

    if not start or not goal or start not in graph_data or goal not in graph_data:
        return jsonify({"error": "Invalid start or goal"}), 400

    dijkstra_path= dijkstra.dijkstra_search(start, goal)
    astar_path= astar.astar_search(start, goal)

    start_lat, start_lon = G.nodes[start]['y'], G.nodes[start]['x']
    m= folium.Map(location=[start_lat, start_lon],zoom_start=1)

    def nodes_to_coords(path):
        return [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]

    # Plot Dijkstra Path in Blue
    folium.PolyLine(nodes_to_coords(dijkstra_path), color="blue", weight=5, opacity=0.7, tooltip="Dijkstra").add_to(m)

    # Plot A* Path in Red
    folium.PolyLine(nodes_to_coords(astar_path), color="red", weight=5, opacity=0.7, tooltip="A*").add_to(m)

    # Mark Start and Goal
    folium.Marker(location=[start_lat, start_lon], popup="Start", icon=folium.Icon(color="green")).add_to(m)
    goal_lat, goal_lon = G.nodes[goal]['y'], G.nodes[goal]['x']
    folium.Marker(location=[goal_lat, goal_lon], popup="Goal", icon=folium.Icon(color="red")).add_to(m)

    # Save map to HTML and return it
    map_path = "static/pathfinding_map.html"
    m.save(map_path)

    return render_template("pathfinding/map_template.html")