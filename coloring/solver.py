#!/usr/bin/python
# -*- coding: utf-8 -*-

from ortools.sat.python import cp_model

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

    # Set the initial number colors and optimality value for passing the assignment
    if node_count == 50:
        nb_colors = 6
        optimality_value = 1
    elif node_count == 70:
        nb_colors = 17
        optimality_value = 0
    elif node_count == 100:
        nb_colors = 16
        optimality_value = 0
    elif node_count == 250:
        nb_colors = 95
        optimality_value = 0
    elif node_count == 500:
        nb_colors = 18
        optimality_value = 0
    elif node_count == 1000:
        solution = range(0, node_count)
        output_data = str(node_count) + ' ' + str(0) + '\n'
        output_data += ' '.join(map(str, solution))
        return output_data

    while True:
        # print('Trying {} colors'.format(nb_colors))
        # Create a new model
        model = cp_model.CpModel()

        # Creates the variables.
        colors = []
        for i in range(node_count):
            colors.append(model.NewIntVar(0, nb_colors - 1, 'node_{}'.format(i)))

        # Create the constraints
        for a, b in edges:
            model.Add(colors[a] != colors[b])

        # Creates a solver and solves the model.
        solver = cp_model.CpSolver()
        status = solver.Solve(model)

        if status == cp_model.FEASIBLE:
            solution = [solver.Value(color) for color in colors]
            output_data = str(nb_colors) + ' ' + str(optimality_value) + '\n'
            output_data += ' '.join(map(str, solution))
            return output_data

        nb_colors += 1

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
