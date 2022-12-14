import heapq
import time
import time as timer

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    if paths is None:
        return None
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent, goal_loc):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = {}
    max_t_step = 0

    for constraint in constraints:
        # print(constraints)
        if constraint['agent'] == agent:
            if constraint['loc'][0] == goal_loc:
                if constraint['t_step'] > max_t_step:
                    max_t_step = constraint['t_step']
            constraint_table.setdefault(constraint['t_step'], []).append(constraint['loc'])
    #print(agent, constraint_table)
    return constraint_table, max_t_step


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if not constraint_table:
        return False
    try:

        for entry in constraint_table[next_time]:
            # print(next_time, constraint_table[next_time], next_loc, curr_loc, entry, len(entry))

            try:
                if len(entry[0][0]) == 2:

                    if (entry[0][0] == curr_loc) & (entry[0][1] == next_loc):
                        return True
            except TypeError:
                #print(curr_loc, next_loc, entry[0])
                if entry[0] == next_loc:
                    #print("rejected")
                    return True
    except KeyError:
        return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], id(node), node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    constraint_table = {}
    start_time = time.time()
    constraint_table, earliest_goal_timestep = build_constraint_table(constraints, agent, goal_loc)
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 't_step': 0}
    push_node(open_list, root)
    closed_list[((root['loc']), root['t_step'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        time_opp = timer.time()-start_time
        if time_opp > 2:
            return None
        if (curr['loc'] == goal_loc) & (curr['t_step'] > earliest_goal_timestep):
            return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            if child_loc[0] > len(my_map)-1 or child_loc[1] > len(my_map[0])-1:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if child_loc[0] < 0 or child_loc[1] < 0:
                continue

            child = {'loc': child_loc[:],
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr.copy(),
                     't_step': curr['t_step'] + 1}

            if is_constrained(curr['loc'], child['loc'], child['t_step'], constraint_table):
                continue

            if (child['loc']) in closed_list:
                existing_node = closed_list[(child['loc'])]
                if compare_nodes(child, existing_node):
                    closed_list[((root['loc']), root['t_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[((root['loc']), root['t_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
