from flask import Blueprint, request, jsonify, render_template
import time
from algorithms.astar import Astar
from algorithms.dijkstra import Dijkstra
from helpers.graph import Graph

#graph blueprint
pathfinding_bp= Blueprint('pathfinding', __name__)

#test graph
graph_data={
    "A": {"B": 3, "C": 3},
    "B": {"A": 3, "D": 3.5, "E": 2.8},
    "C": {"A": 3, "E": 2.8, "F": 3.5},
    "D": {"B": 3.5, "E": 3.1, "G": 10},
    "E": {"B": 2.8, "C": 2.8, "D": 3.1, "G": 7},
    "F": {"G": 2.5, "C": 3.5},
    "G": {"F": 2.5, "E": 7, "D": 10}
}

coords = {
    "A": (0, 0), "B": (1, 2), "C": (2, 0),
    "D": (3, 4), "E": (2, 3), "F": (4, 1), "G": (5, 5)
}

graph_obj = Graph(graph_data,coords) # create graph
astar = Astar(graph_obj)
dijkstra = Dijkstra(graph_obj)

@pathfinding_bp.route('/find_path', methods=['GET']) # API
def find_path():
    start = request.args.get('start')
    goal = request.args.get('goal')

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

@pathfinding_bp.route('/visualizer')
def pathfinding_page():
    return render_template('pathfinding/index.html')  # serve visualisation of algorithms page
