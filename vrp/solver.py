#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

def length(customer1, customer2):
    return math.sqrt((customer1.x - customer2.x)**2 + (customer1.y - customer2.y)**2)

def create_distance_callback(customers):
    # Creates callback to return distance between customers.
    _distances = {}
    num_customers = len(customers)
    for from_node in range(num_customers):
        _distances[from_node] = {}
        for to_node in range(num_customers):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            else:
                _distances[from_node][to_node] = length(customers[from_node], customers[to_node])

    def distance_callback(from_node, to_node):
        return _distances[from_node][to_node]

    return distance_callback

def create_demand_callback(customers):
    # Creates callback to get demands at each location.
    def demand_callback(from_node, to_node):
        return customers[from_node].demand
    return demand_callback

def add_capacity_constraints(routing, demand_callback, vehicle_capacities):
    # Adds capacity constraint
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback,
        0, # null capacity slack
        vehicle_capacities,
        True, # start cumul to zero
        capacity)

def trivial_solution(customer_count, vehicle_count, vehicle_capacity, customers):
    #the depot is always the first customer in the input
    depot = customers[0]


    # build a trivial solution
    # assign customers to vehicles starting by the largest customer demands
    vehicle_tours = []

    remaining_customers = set(customers)
    remaining_customers.remove(depot)

    for v in range(0, vehicle_count):
        # print "Start Vehicle: ",v
        vehicle_tours.append([])
        capacity_remaining = vehicle_capacity
        while sum([capacity_remaining >= customer.demand for customer in remaining_customers]) > 0:
            used = set()
            order = sorted(remaining_customers, key=lambda customer: -customer.demand)
            for customer in order:
                if capacity_remaining >= customer.demand:
                    capacity_remaining -= customer.demand
                    vehicle_tours[v].append(customer)
                    # print '   add', ci, capacity_remaining
                    used.add(customer)
            remaining_customers -= used

    # checks that the number of customers served is correct
    assert sum([len(v) for v in vehicle_tours]) == len(customers) - 1

    # calculate the cost of the solution; for each vehicle the length of the route
    obj = 0
    for v in range(0, vehicle_count):
        vehicle_tour = vehicle_tours[v]
        if len(vehicle_tour) > 0:
            obj += length(depot,vehicle_tour[0])
            for i in range(0, len(vehicle_tour)-1):
                obj += length(vehicle_tour[i],vehicle_tour[i+1])
            obj += length(vehicle_tour[-1],depot)

    # prepare the solution in the specified output format
    outputData = '%.2f' % obj + ' ' + str(0) + '\n'
    for v in range(0, vehicle_count):
        outputData += str(depot.index) + ' ' + ' '.join([str(customer.index) for customer in vehicle_tours[v]]) + ' ' + str(depot.index) + '\n'

    return outputData

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])

    customers = []
    for i in range(1, customer_count+1):
        line = lines[i]
        parts = line.split()
        customers.append(Customer(i-1, int(parts[0]), float(parts[1]), float(parts[2])))

    # Get the trivial solution
    trivial_solution_output = trivial_solution(customer_count, vehicle_count, vehicle_capacity, customers)

    # Use Google OR-Tool for CVRP
    routing = pywrapcp.RoutingModel(customer_count, vehicle_count, 0)
    distance_callback = create_distance_callback(customers)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
    # Add Capacity constraint
    demand_callback = create_demand_callback(customers)
    add_capacity_constraints(routing, demand_callback, [vehicle_capacity] * vehicle_count)
    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        total_dist = 0.0
        solution_routes = []
        for vehicle_id in range(vehicle_count):
            solution_route = []
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                solution_route.append(node_index)
                next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
                total_dist += length(customers[node_index], customers[next_node_index])
                index = assignment.Value(routing.NextVar(index))

            node_index = routing.IndexToNode(index)
            solution_route.append(node_index)
            solution_routes.append(solution_route[1:-1])

        # prepare the solution in the specified output format
        outputData = '%.2f' % total_dist + ' ' + str(0) + '\n'
        for v in range(0, vehicle_count):
            outputData += str(0) + ' ' + ' '.join([str(customer) for customer in solution_routes[v]]) + ' ' + str(0) + '\n'
        return outputData

import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:

        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')
