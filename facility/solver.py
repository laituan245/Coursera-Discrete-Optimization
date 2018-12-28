#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple
from ortools.linear_solver import pywraplp
import math

Point = namedtuple("Point", ['x', 'y'])
Facility = namedtuple("Facility", ['index', 'setup_cost', 'capacity', 'location'])
Customer = namedtuple("Customer", ['index', 'demand', 'location'])

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def _trivial_solution(facilities, customers):
    facility_count = len(facilities)
    customer_count = len(customers)
    # build a trivial solution
    # pack the facilities one by one until all the customers are served
    solution = [-1]*len(customers)
    capacity_remaining = [f.capacity for f in facilities]

    facility_index = 0
    for customer in customers:
        if capacity_remaining[facility_index] >= customer.demand:
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand
        else:
            facility_index += 1
            assert capacity_remaining[facility_index] >= customer.demand
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand

    used = [0]*len(facilities)
    for facility_index in solution:
        used[facility_index] = 1

    # calculate the cost of the solution
    obj = sum([f.setup_cost*used[f.index] for f in facilities])
    for customer in customers:
        obj += length(customer.location, facilities[solution[customer.index]].location)

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    facility_count = int(parts[0])
    customer_count = int(parts[1])

    facilities = []
    for i in range(1, facility_count+1):
        parts = lines[i].split()
        facilities.append(Facility(i-1, float(parts[0]), int(parts[1]), Point(float(parts[2]), float(parts[3])) ))

    customers = []
    for i in range(facility_count+1, facility_count+1+customer_count):
        parts = lines[i].split()
        customers.append(Customer(i-1-facility_count, int(parts[0]), Point(float(parts[1]), float(parts[2]))))

    if facility_count > 250:
        return _trivial_solution(facilities, customers)

    # Define MILP Model
    solver = pywraplp.Solver('MILP Solver', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    solver.SetTimeLimit(100000)

    # Define the decision variables
    is_open_vars = []
    for i in range(facility_count):
        is_open_vars.append(solver.IntVar(0.0, 1.0, 'open_{}'.format(i)))
    is_serve_vars = [[] for i in range(facility_count)]
    for i in range(facility_count):
        for j in range(customer_count):
            is_serve_vars[i].append(solver.IntVar(0.0, 1.0, 'serve_{}_{}'.format(i, j)))

    # Define the constraints
    # Constraint 1. Should not exceed capacity of a facility
    for i in range(facility_count):
        constraint = solver.Constraint(-solver.infinity(), 0)
        for j in range(customer_count):
            constraint.SetCoefficient(is_serve_vars[i][j], customers[j].demand)
        constraint.SetCoefficient(is_open_vars[i], -facilities[i].capacity)

    # Constraint 2. Every customer has to be served by exactly 1 facility
    for j in range(customer_count):
        constraint = solver.Constraint(1, 1)
        for i in range(facility_count):
            constraint.SetCoefficient(is_serve_vars[i][j], 1)

    # Set up objective
    objective = solver.Objective()
    for i in range(facility_count):
        objective.SetCoefficient(is_open_vars[i], facilities[i].setup_cost)
    for i in range(facility_count):
        for j in range(customer_count):
            objective.SetCoefficient(is_serve_vars[i][j], length(facilities[i].location, customers[j].location))
    objective.SetMinimization()
    result_status = solver.Solve()

    if not (result_status == pywraplp.Solver.OPTIMAL or result_status == pywraplp.Solver.FEASIBLE):
        return _trivial_solution(facilities, customers)

    obj = solver.Objective().Value()
    solution = []
    for j in range(customer_count):
        for i in range(facility_count):
            if is_serve_vars[i][j].solution_value() == 1:
                solution.append(i)
                break
    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))

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
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/fl_16_2)')
