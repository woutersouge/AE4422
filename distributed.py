"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed

from cbs import detect_collision, detect_collisions


class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []

        # T.B.D.

    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 

        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time

        look_steps = 2  # <------------------------------------------------------------ CHANGE LOOK FORWARD STEPS HERE

        # Create agent objects with AircraftDistributed class
        agentlist = []
        moves = [[] for Null in range(self.num_of_agents)]
        vals = [1 for Null in range(self.num_of_agents)]

        for i in range(self.num_of_agents):
            heur = compute_heuristics(self.my_map, self.goals[i])
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], heur, i, False)
            agentlist.append(newAgent)
            moves[i].append(self.starts[i])

        while not all(item == 0 for item in vals):
            ten = [{} for _ in range(self.num_of_agents)]
            curr = [{} for _ in range(self.num_of_agents)]
            for j in range(self.num_of_agents):
                #print(j)
                ten[j]['loc'], ten[j]['val'], curr[j]['loc'] = agentlist[j].calc_next(curr, look_steps)
            places = [d['loc'] for d in curr]
            print_mapf_instance(self.my_map, places, self.goals)
            ten_loc = [d['loc'] for d in ten]

            coll = False

            for location in [d['loc'] for d in ten]:
                if ten_loc.count(location) > 1:
                    coll = True
                    break
                else:
                    coll = False

                if location in [d['loc'] for d in curr]:
                    if ten[[d['loc'] for d in curr].index(location)]['loc'] == \
                            curr[[d['loc'] for d in ten].index(location)]['loc']:
                        # print([d['loc'] for d in curr].index(location),[d['loc'] for d in ten].index(location))
                        if [d['loc'] for d in curr].index(location) != [d['loc'] for d in ten].index(location):
                            coll = True
                            break

            while coll:
                for k in range(self.num_of_agents):
                    ten = agentlist[k].solve_coll(curr, ten, look_steps)

                    ten_loc = [d['loc'] for d in ten]

                    coll = False

                    for location in ten_loc:
                        # print(ten_loc.count(location))
                        if ten_loc.count(location) > 1:
                            coll = True
                            break
                        else:
                            coll = False

                        if location in [d['loc'] for d in curr]:
                            if ten[[d['loc'] for d in curr].index(location)]['loc'] == \
                                    curr[[d['loc'] for d in ten].index(location)]['loc']:

                                if [d['loc'] for d in curr].index(location) != [d['loc'] for d in ten].index(location):
                                    # print([d['loc'] for d in curr].index(location),
                                    #      [d['loc'] for d in ten].index(location))
                                    coll = True
                                    break
                    if not coll:
                        break

            for l in range(self.num_of_agents):
                moves[l].append(ten[l]['loc'])
                agentlist[l].update_loc(ten[l]['loc'])

            vals = [d['val'] for d in ten]

        result = moves

        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(
            get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)

        return result, self.CPU_time  # Hint: this should be the final result of the distributed planning (visualization is done after planning)


def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and
    a number for the start location of each agent.

    Example:
        @ @ @ @ @ @ @
        @ 0 1 . . . @
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
    """
    print('Start locations')
    print_locations(my_map, starts)



def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)