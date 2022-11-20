import time as timer
import heapq
import itertools
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    for i in range(max(len(path1), len(path2))):
        if get_location(path1, i) == get_location(path2, i):
            return get_location(path1, i), i
        if (get_location(path1, i) == get_location(path2, i + 1)) & (
                get_location(path1, i + 1) == get_location(path2, i)):
            return (get_location(path1, i), get_location(path1, i + 1)), i + 1

    return None, None


def detect_collisions(paths):
    collisions = list()

    for comb in itertools.combinations(range(len(paths)), 2):
        coll, t = detect_collision(paths[comb[0]], paths[comb[1]])
        if (coll is None) or (t is None):
            continue
        collisions.append({'a1': comb[0], 'a2': comb[1], 'loc': [coll], 't_step': t})

    return collisions


def standard_splitting(collision):
    constraints = list()
    try:

        if len(collision['loc'][0][0]) == 2:
            constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']})
            constraints.append({'agent': collision['a2'], 'loc': [(collision['loc'][0][1], collision['loc'][0][0])],
                                't_step': collision['t_step']})

    except TypeError:
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 't_step': collision['t_step']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 't_step': collision['t_step']})

    return constraints


def disjoint_splitting(collision):

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
        
        while len(self.open_list) > 0:
            curr = self.pop_node()
            print(self.CPU_time)
            if not curr['collisions']: 
                self.CPU_time = timer.time() - self.start_time   
                return curr['paths'], self.CPU_time

            constraints2 = standard_splitting(curr['collisions'][0])
            for constraint2 in constraints2:
                self.CPU_time = timer.time() - self.start_time
                print(self.CPU_time)
                if self.CPU_time > 2:
                    return None, 'Non Solvable'
                conn = curr['constraints'][:]
                conn.append(constraint2)
                child2 = {'cost': 0, 'constraints': conn, 'paths': curr['paths'][:], 'collisions': []}
                agent = child2['constraints'][-1]['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                              agent, conn)
                if path is None:
                    break

                if path:
                    child2['paths'][agent] = path[:]

                    child2['collisions'] = detect_collisions(child2['paths'])
                    child2['cost'] = get_sum_of_cost(child2['paths'])
                    self.push_node(child2)

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        return None, self.CPU_time
