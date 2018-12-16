#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import namedtuple
Item = namedtuple("Item", ['index', 'value', 'weight'])

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    firstLine = lines[0].split()
    item_count = int(firstLine[0])
    capacity = int(firstLine[1])

    items = []

    for i in range(1, item_count+1):
        line = lines[i]
        parts = line.split()
        items.append(Item(i-1, int(parts[0]), int(parts[1])))

    # ============== Dynamic Programming Approach ==============
    dp = [[0 for i in range(capacity+1)] for j in range(len(items) + 1)]


    for i in range(1, len(items) + 1):
        current_item = items[i-1]
        for _capacity in range(capacity+1):
            dp[i][_capacity] = dp[i-1][_capacity]
            if _capacity >= current_item.weight:
                dp[i][_capacity] = max(dp[i][_capacity],
                                       current_item.value + dp[i-1][_capacity-current_item.weight])
    value = dp[len(items)][capacity]

    taken = [0] * len(items)
    current_y = capacity
    for i in range(len(items), 0, -1):
        if dp[i][current_y] != dp[i-1][current_y]:
            taken[i-1] = 1
            current_y = current_y - items[i-1].weight

    is_optimal = 1

    # prepare the solution in the specified output format
    output_data = str(value) + ' ' + str(is_optimal) + '\n'
    output_data += ' '.join(map(str, taken))
    return output_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/ks_4_0)')
