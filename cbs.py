import time as timer
import heapq
import itertools
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for i in range(max(len(path1), len(path2))):
        if get_location(path1, i) == get_location(path2, i):
            return get_location(path1, i), i
        if (get_location(path1, i) == get_location(path2, i + 1)) & (
                get_location(path1, i + 1) == get_location(path2, i)):
            return (get_location(path1, i), get_location(path1, i + 1)), i + 1

    return None, None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = list()

    for comb in itertools.combinations(range(len(paths)), 2):
        coll, t = detect_collision(paths[comb[0]], paths[comb[1]])
        # print(coll, t)
        if (coll is None) or (t is None):
            continue
        collisions.append({'a1': comb[0], 'a2': comb[1], 'loc': [coll], 't_step': t})

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = list()
    try:

        # print(constraints[-1]['loc'])
        if len(collision['loc'][0][0]) == 2:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']})
            constraints.append({'agent': collision['a2'], 'loc': [(collision['loc'][0][1], collision['loc'][0][0])],
                                't_step': collision['t_step']})
            # print(constraints[-1]['loc'])

    except TypeError:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 't_step': collision['t_step']})

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.start_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        # print(len(root['paths']))
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #    print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node())
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit       
        
        while len(self.open_list) > 0:
            curr = self.pop_node()
            print(self.CPU_time)
            if not curr['collisions']: 
                self.CPU_time = timer.time() - self.start_time   
                return curr['paths'], self.CPU_time

            constraints2 = standard_splitting(curr['collisions'][0])
            # print(constraints2)
            for constraint2 in constraints2:
                self.CPU_time = timer.time() - self.start_time
                print(self.CPU_time)
                if self.CPU_time > 2:
                    return None, 'Non Solvable'
                conn = curr['constraints'][:]
                conn.append(constraint2)
                # print(curr['paths'])
                child2 = {'cost': 0, 'constraints': conn, 'paths': curr['paths'][:], 'collisions': []}
                # print(child2['constraints'])
                agent = child2['constraints'][-1]['agent']
                # print(agent)
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                              agent, conn)
                if path is None:
                    break

                if path:
                    child2['paths'][agent] = path[:]

                    child2['collisions'] = detect_collisions(child2['paths'])
                    child2['cost'] = get_sum_of_cost(child2['paths'])
                    # print(child2)
                    # print(child2)
                    self.push_node(child2)

        # self.print_results(root)

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        # print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        return None, self.CPU_time
