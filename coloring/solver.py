#!/usr/bin/python
# -*- coding: utf-8 -*-
from ortools.linear_solver import pywraplp

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    first_line = lines[0].split()
    node_count = int(first_line[0])
    edge_count = int(first_line[1])

    edges = []
    for i in range(1, edge_count + 1):
        line = lines[i]
        parts = line.split()
        edges.append((int(parts[0]), int(parts[1])))

    # Define the MIP solver
    solver = pywraplp.Solver('SolveIntegerProblem',
                              pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    max_colors = 8


    # Define the binary decision variables
    # x_i_0 indicates whether the node i is colored red or not
    # x_i_1 indicates whether the node i is colored blue or not
    # x_i_2 indicates whether the node i is colored green or not
    # x_i_3 indicates whether the node i is colored white or not
    decision_variables = [[solver.IntVar(0.0, 1.0, 'x_{}_{}'.format(i,j)) for j in range(max_colors)] for i in range(node_count)]

    # Definte the objective function
    nb_colors = solver.IntVar(0.0, max_colors-1, 'nb_colors')
    objective = solver.Objective()
    objective.SetCoefficient(nb_colors, 1)
    objective.SetMinimization()

    # Define the constraints

    # 1. Each node has 1 color
    for node in range(node_count):
        constraint = solver.Constraint(1, 1)
        for j in range(max_colors):
            constraint.SetCoefficient(decision_variables[node][j], 1)


    # 2. Adjacent Node needs to have different colors
    for (nodeA, nodeB) in edges:
        for j in range(max_colors):
            constraint = solver.Constraint(-solver.infinity(), 1)
            constraint.SetCoefficient(decision_variables[nodeA][j], 1)
            constraint.SetCoefficient(decision_variables[nodeB][j], 1)

    # 3. Final objective
    for node in range(node_count):
        for j in range(max_colors):
            constraint = solver.Constraint(-solver.infinity(), 0)
            constraint.SetCoefficient(decision_variables[node][j], j)
            constraint.SetCoefficient(nb_colors, -1)

    # Solve
    result_status = solver.Solve()
    # The problem has an optimal solution.
    assert result_status == pywraplp.Solver.OPTIMAL
    # Extract the solution
    solution = []
    for i in range(node_count):
        for j in range(max_colors):
            if decision_variables[i][j].solution_value() > 0:
                solution.append(j)
                break

    # prepare the solution in the specified output format
    output_data = str(node_count) + ' ' + str(1) + '\n'
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
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/gc_4_1)')
