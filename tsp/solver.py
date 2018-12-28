#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

Point = namedtuple("Point", ['x', 'y'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def create_distance_callback(points):
    def distance_callback(from_node, to_node):
        return int(100 * length(points[from_node], points[to_node]))

    return distance_callback

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    nodeCount = int(lines[0])

    points = []
    for i in range(1, nodeCount+1):
        line = lines[i]
        parts = line.split()
        points.append(Point(float(parts[0]), float(parts[1])))

    # If the problem size is too large, use trivial solution
    if len(points) > 10000:
        solution = range(0, nodeCount)
        obj = length(points[solution[-1]], points[solution[0]])
        for index in range(0, nodeCount-1):
            obj += length(points[solution[index]], points[solution[index+1]])
        output_data = '%.2f' % obj + ' ' + str(0) + '\n'
        output_data += ' '.join(map(str, solution))
        return output_data

    # Declare the solver
    routing = pywrapcp.RoutingModel(len(points), 1, 0)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

    # If the problem size is small, use guided local search
    if len(points) < 5000:
        search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit_ms = 300000

    # Create the distance callback.
    dist_callback = create_distance_callback(points)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        results = []
        route_number = 0
        index = routing.Start(route_number) # Index of the variable for the starting node.
        while not routing.IsEnd(index):
            results.append(index)
            index = assignment.Value(routing.NextVar(index))

        obj = 0.0
        for i in range(1, len(results)):
            obj += length(points[results[i-1]], points[results[i]])
        obj += length(points[results[-1]], points[0])
        output_data = '%.2f' % obj + ' ' + str(0) + '\n'
        output_data += ' '.join(map(str, results))
        return output_data
import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/tsp_51_1)')
