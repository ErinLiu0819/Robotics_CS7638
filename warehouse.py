######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

import math
from typing import final

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class DeliveryPlanner_PartA:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

      plan_delivery(self, debug = False):
       Stubbed out below.  You may not change the method signature
        as it will be called directly by the autograder but you
        may modify the internals as needed.

      __init__:
        Required to initialize the class.  Signature can NOT be changed.
        Basic template starter code is provided.  You may choose to
        use this starter code or modify and replace it based on
        your own solution.
    """

    def __init__(self, warehouse_viewer, dropzone_location, todo, box_locations):

        self.warehouse_viewer = warehouse_viewer
        self.dropzone_location = dropzone_location
        self.todo = todo
        self.box_locations = box_locations

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def plan_delivery(self, debug=True):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # the heuristic function is learned from https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#heuristics-for-grid-maps
        delta = [[-1, 0], [0, 1], [1, 0], [0, -1], [-1, 1], [-1, -1], [1, 1], [1, -1]]
        delta_name = ['n', 'e', 's', 'w', 'ne', 'nw', 'se', 'sw']
        cost = [2, 2, 2, 2, 3, 3, 3, 3]
        pickup_cost = 4
        drop_cost = 2
        D = 2
        D2 = 3

        moves = []
        init = self.dropzone_location

        for box in self.todo:
            goal = self.box_locations[box]

            # Go to the box location
            open_list = []
            x, y = init
            g = 0
            dx = abs(x - goal[0])
            dy = abs(y - goal[1])
            h = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
            f = g + h
            open_list.append([f, g, h, x, y, []])  # Last element [] is path to here
            viewed = set()
            final_path = []

            while open_list:
                open_list.sort(reverse=True)
                f, g, h, x, y, path_to_here = open_list.pop()

                if (x, y) in viewed:
                    continue

                viewed.add((x, y))

                if any((x + dx, y + dy) == goal for dx, dy in delta):
                    final_path = path_to_here + [f"lift {box}"]
                    moves.extend(final_path)
                    init = (x, y)
                    self.warehouse_viewer[goal[0]][goal[1]] = '.'
                    break

                # Explore neighbors
                for i, (delta_x, delta_y) in enumerate(delta):
                    x2, y2 = x + delta_x, y + delta_y

                    # Check boundaries and obstacles
                    try:
                        if self.warehouse_viewer[x2][y2] == '#':
                            continue
                    except IndexError:
                        continue

                    if (x2, y2) in viewed:
                        continue

                    g2 = g + cost[i]
                    dx = abs(x2 - goal[0])
                    dy = abs(y2 - goal[1])
                    h2 = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
                    f2 = g2 + h2
                    new_path = path_to_here + [f"move {delta_name[i]}"]
                    open_list.append([f2, g2, h2, x2, y2, new_path])

            # Go back to the dropzone
            goal = self.dropzone_location
            open_list = []
            x, y = init  # Start from lift location
            g = 0
            dx = abs(x - goal[0])
            dy = abs(y - goal[1])
            h = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
            f = g + h
            open_list.append([f, g, h, x, y, []])
            viewed = set()
            final_path_back = []

            while open_list:
                open_list.sort(reverse=True)
                f, g, h, x, y, path_to_here = open_list.pop()

                if (x, y) in viewed:
                    continue

                viewed.add((x, y))


                for i, (dx, dy) in enumerate(delta):
                    if (x + dx, y + dy) == goal:
                        final_drop_direction = delta_name[i]
                        final_path_back = path_to_here + [f"down {final_drop_direction}"]
                        moves.extend(final_path_back)
                        init = (x, y)
                        break
                else:
                    # Explore neighbors
                    for i, (delta_x, delta_y) in enumerate(delta):
                        x2, y2 = x + delta_x, y + delta_y

                        # Check boundaries and obstacles
                        try:
                            if self.warehouse_viewer[x2][y2] == '#':
                                continue
                        except IndexError:
                            continue

                        if (x2, y2) in viewed:
                            continue

                        g2 = g + cost[i]
                        dx = abs(x2 - goal[0])
                        dy = abs(y2 - goal[1])
                        h2 = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
                        f2 = g2 + h2
                        new_path = path_to_here + [f"move {delta_name[i]}"]
                        open_list.append([f2, g2, h2, x2, y2, new_path])

                if final_path_back:
                    break

        if debug:
            for i in range(len(moves)):
                print(moves[i])

        return moves


class DeliveryPlanner_PartB:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)


    def generate_policies(self, debug=True):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """
        delta = [[-1, 0], [0, 1], [1, 0], [0, -1], [-1, 1], [-1, -1], [1, 1], [1, -1]]
        delta_name = ['n', 'e', 's', 'w', 'ne', 'nw', 'se', 'sw']
        cost = [2, 2, 2, 2, 3, 3, 3, 3]
        pickup_cost = 4
        drop_cost = 2

        rows, cols = len(self.warehouse_state), len(self.warehouse_state[0])

        to_box_value = [[9999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        to_box_policy = [[' ' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        deliver_value = [[9999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        deliver_policy = [[' ' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        box_location = self.boxes['1']
        dropzone = self.dropzone

        # find to way to the box location
        change = True
        while change:
            change = False
            for x in range(rows):
                for y in range(cols):
                    if (x,y) == box_location:
                        if to_box_value[x][y] > 0:
                            to_box_value[x][y] = 0
                            change = True
                            to_box_policy[x][y] = 'B'
                    elif self.warehouse_state[x][y] != '#':
                        for i, (dx, dy) in enumerate(delta):
                            x2, y2 = x + dx, y + dy
                            if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                if (x2, y2) == box_location:
                                    v2 = to_box_value[x2][y2] + pickup_cost + self.warehouse_cost[x2][y2]
                                    if v2 < to_box_value[x][y]:
                                        change = True
                                        to_box_value[x][y] = v2
                                        to_box_policy[x][y] = 'lift 1'
                                else:
                                    v2 = to_box_value[x2][y2] + cost[i] + self.warehouse_cost[x2][y2]
                                    if v2 < to_box_value[x][y]:
                                        change = True
                                        to_box_value[x][y] = v2
                                        to_box_policy[x][y] = f'move {delta_name[i]}'
                    elif self.warehouse_state[x][y] == '#':
                        to_box_policy[x][y] = '-1'


        # back to dropzone
        change = True
        while change:
            change = False
            for x in range(rows):
                for y in range(cols):
                    if (x, y) == dropzone:
                        if deliver_value[x][y] > 0:
                            deliver_value[x][y] = 0
                            change = True
                            for i, (dx, dy) in enumerate(delta):
                                x2, y2 = x + dx, y + dy
                                if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                    deliver_policy[x][y] = f'move {delta_name[i]}'

                    elif self.warehouse_state[x][y] != '#':
                        for i, (dx, dy) in enumerate(delta):
                            x2, y2 = x + dx, y + dy
                            if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                if (x2, y2) == dropzone:
                                    v2 = deliver_value[x2][y2] + drop_cost + self.warehouse_cost[x2][y2]
                                    if v2 < deliver_value[x][y]:
                                        change = True
                                        deliver_value[x][y] = v2
                                        deliver_policy[x][y] = f'down {delta_name[i]}'
                                else:
                                    v2 = deliver_value[x2][y2] + cost[i] + self.warehouse_cost[x2][y2]
                                    if v2 < deliver_value[x][y]:
                                        change = True
                                        deliver_value[x][y] = v2
                                        deliver_policy[x][y] = f'move {delta_name[i]}'
                    elif self.warehouse_state[x][y] == '#':
                        deliver_policy[x][y] = '-1'


        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return (to_box_policy, deliver_policy)


class DeliveryPlanner_PartC:
    """
    [Doc string same as part B]
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.stochastic_probabilities = stochastic_probabilities


    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def generate_policies(self, debug=True):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        delta = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
        delta_name = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']
        cost = [2, 3, 2, 3, 2, 3, 2, 3]
        pickup_cost = 4
        drop_cost = 2

        rows, cols = len(self.warehouse_state), len(self.warehouse_state[0])

        to_box_values = [[9999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        to_box_policy = [[' ' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        to_zone_values = [[9999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        to_zone_policy = [[' ' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]

        # Extract probabilities for stochastic movements
        intended_prob = self.stochastic_probabilities["as_intended"]
        slanted_prob = self.stochastic_probabilities["slanted"]
        sideways_prob = self.stochastic_probabilities["sideways"]

        box_location = self.boxes['1']
        dropzone = self.dropzone

        # Policy for reaching the box
        change = True
        while change:
            change = False
            for x in range(rows):
                for y in range(cols):
                    if (x, y) == box_location:
                        if to_box_values[x][y] > 0:
                            to_box_values[x][y] = 0
                            change = True
                            to_box_policy[x][y] = 'B'
                            for i, (dx, dy) in enumerate(delta):
                                x2, y2 = x + dx, y + dy
                                if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                    to_zone_policy[x][y] = f'move {delta_name[i]}'

                    elif self.warehouse_state[x][y] != '#':
                        for i in range(len(delta)):
                            x2, y2 = x + delta[i][0], y + delta[i][1]
                            # if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                            if (x2, y2) == box_location:
                                v2 = to_box_values[x2][y2] + pickup_cost + cost[i] + self.warehouse_cost[x2][y2]
                                if v2 < to_box_values[x][y]:
                                    change = True
                                    to_box_values[x][y] = v2
                                    to_box_policy[x][y] = 'lift 1'

                            v2 = 0
                            for j in [-2, -1, 0, 1, 2]:  # Account for sideways and slanted movements
                                i2 = (i + j) % len(delta)
                                x2, y2 = x + delta[i2][0], y + delta[i2][1]
                                movement_cost = cost[i2]

                                # Determine probability based on the type of move
                                if j == 0:
                                    prob = intended_prob
                                elif abs(j) == 1:
                                    prob = slanted_prob
                                else:
                                    prob = sideways_prob

                                # Check if the cell is available
                                if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                    v2 += prob * (to_box_values[x2][y2] + movement_cost + self.warehouse_cost[x2][y2])
                                else:
                                    # Add penalty cost
                                    v2 += prob * (to_box_values[x][y] + self.warehouse_cost[x][y] + 100) # + self.warehouse_cost[x][y]

                            if v2 < to_box_values[x][y]:
                                change = True
                                to_box_values[x][y] = v2
                                to_box_policy[x][y] = f'move {delta_name[i]}'

                    elif self.warehouse_state[x][y] == '#':
                        to_box_policy[x][y] = '-1'

        # Policy for returning to the dropzone
        change = True
        while change:
            change = False
            for x in range(rows):
                for y in range(cols):
                    if (x, y) == dropzone:
                        if to_zone_values[x][y] > 0:
                            to_zone_values[x][y] = 0
                            change = True
                            for i, (dx, dy) in enumerate(delta):
                                x2, y2 = x + dx, y + dy
                                if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                    to_zone_policy[x][y] = f'move {delta_name[i]}'

                    elif self.warehouse_state[x][y] != '#':
                        for i in range(len(delta)):
                            x2, y2 = x + delta[i][0], y + delta[i][1]
                            # if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                            if (x2, y2) == dropzone:
                                v2 = to_zone_values[x2][y2] + drop_cost + self.warehouse_cost[x2][y2]
                                if v2 < to_zone_values[x][y]:
                                    change = True
                                    to_zone_values[x][y] = v2
                                    to_zone_policy[x][y] = f'down {delta_name[i]}'
                            v2 = 0
                            for j in [-2, -1, 0, 1, 2]:  # Account for sideways and slanted movements
                                i2 = (i + j) % len(delta)
                                x2, y2 = x + delta[i2][0], y + delta[i2][1]
                                movement_cost = cost[i2]

                                # Determine probability based on the type of move
                                if j == 0:
                                    prob = intended_prob
                                elif abs(j) == 1:
                                    prob = slanted_prob
                                else:
                                    prob = sideways_prob

                                # Check if the cell is available
                                if 0 <= x2 < rows and 0 <= y2 < cols and self.warehouse_state[x2][y2] != '#':
                                    v2 += prob * (to_zone_values[x2][y2] + movement_cost + self.warehouse_cost[x2][y2])
                                else:
                                    # Add penalty cost
                                    v2 += prob * (movement_cost + 600)

                            if v2 < to_zone_values[x][y]:
                                change = True
                                to_zone_values[x][y] = v2
                                to_zone_policy[x][y] = f'move {delta_name[i]}'

                    elif self.warehouse_state[x][y] == '#':
                        to_zone_policy[x][y] = '-1'

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.

        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith324).
    whoami = 'xliu3019'
    return whoami


if __name__ == "__main__":
    """
    You may execute this file to develop and test the search algorithm prior to running
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    print('\n~~~ Testing for part A: ~~~\n')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    # testcase 1
    warehouse = [
        '######',
        '#....#',
        '#.1#2#',
        '#..#.#',
        '#...@#',
        '######',
    ]
    todo = list('12')
    benchmark_cost = 23
    viewed_cell_count_threshold = 20
    dropzone = (4,4)
    box_locations = {
        '1': (2,2),
        '2': (2,4),
    }
    # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, dropzone, todo, box_locations)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\n~~~ Testing for part B: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.generate_policies(debug=True)

    # Testing for Part C
    # testcase 1
    print('\n~~~ Testing for part C: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    stochastic_probabilities = {
        'as_intended': .70,
        'slanted': .1,
        'sideways': .05,
    }

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partC.generate_policies(debug=True)
