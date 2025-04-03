import folium
import osmnx.graph
from flask import Blueprint, request, jsonify, render_template
import time
from algorithms.astar import Astar
from algorithms.dijkstra import Dijkstra
from helpers.graph import Graph
import osmnx as ox

from helpers.map import load_place, convert_to_graph

pathfinding_bp= Blueprint('pathfinding', __name__)

place_name = "Attica, Greece"
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

    nearest_node = ox.distance.nearest_nodes(G, lon, lat)  # get closest OSM node

    return jsonify({"node_id": nearest_node})

@pathfinding_bp.route('/find_path',methods=['GET'])
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

    pathfinding_result = find_path().json
    if "error" in pathfinding_result:
        return jsonify(pathfinding_result), 400

    dijkstra_path= pathfinding_result["dijkstra"]["path"]
    astar_path= pathfinding_result["astar"]["path"]
    dijkstra_time= pathfinding_result["dijkstra"]["time"]
    astar_time= pathfinding_result["astar"]["time"]

    def nodes_to_coords(path):
        return [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]

    def draw_path(path, line_color, time, name):
        start_lat, start_lon = G.nodes[path[0]]['y'], G.nodes[path[0]]['x']
        m= folium.Map(location=[start_lat, start_lon],zoom_start=15)

        folium.PolyLine(nodes_to_coords(path), color= line_color, weight=5, opacity=0.4).add_to(m)

        # mark start goal
        folium.Marker(location=[start_lat, start_lon], popup="Start", icon=folium.Icon(color="green")).add_to(m)
        goal_lat, goal_lon = G.nodes[goal]['y'], G.nodes[goal]['x']
        folium.Marker(location=[goal_lat, goal_lon], popup="Goal", icon=folium.Icon(color="red")).add_to(m)
        m.fit_bounds([[start_lat, start_lon], [goal_lat, goal_lon]], padding=(50, 50))

        # save map to HTML and return it
        map_path = "static/pathfinding_map_"+name+".html"
        m.save(map_path)

    draw_path(dijkstra_path,"blue",dijkstra_time,"Dijkstra")
    draw_path(astar_path,"red",astar_time,"Astar")
    return render_template("pathfinding/map_template.html",dijkstra_time=dijkstra_time,astar_time=astar_time)
@pathfinding_bp.route('/select_points')
def select_points():
    return render_template("pathfinding/point_selection.html")