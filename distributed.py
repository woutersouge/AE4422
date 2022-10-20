"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions

<<<<<<< HEAD
=======

>>>>>>> main
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
<<<<<<< HEAD
        
=======

>>>>>>> main
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
<<<<<<< HEAD
        
        
=======

>>>>>>> main
        # Create agent objects with AircraftDistributed class
        agentlist = []
        moves = [[] for Null in range(self.num_of_agents)]
        vals = [1 for Null in range(self.num_of_agents)]

<<<<<<< HEAD

=======
>>>>>>> main
        for i in range(self.num_of_agents):
            heur = compute_heuristics(self.my_map, self.goals[i])
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], heur, i)
            agentlist.append(newAgent)
            moves[i].append(self.starts[i])

        while not all(item == 0 for item in vals):
            ten = [{} for _ in range(self.num_of_agents)]
<<<<<<< HEAD
            for j in range(self.num_of_agents):
                loc, val = agentlist[j].calc_next()
                ten[j]['val'] = val
                ten[j]['loc'] = loc
                vals[j] = val



                moves[j].append(loc)
                agentlist[j].update_loc(loc)
=======
            curr = [{} for _ in range(self.num_of_agents)]
            for j in range(self.num_of_agents):
                ten[j]['loc'], ten[j]['val'], curr[j]['loc'] = agentlist[j].calc_next(curr)
>>>>>>> main

            ten_loc = [d['loc'] for d in ten]

            coll = False

<<<<<<< HEAD
            for location in ten_loc:
=======
            for location in [d['loc'] for d in ten]:
>>>>>>> main
                if ten_loc.count(location) > 1:
                    coll = True
                    break
                else:
                    coll = False

<<<<<<< HEAD
            while coll:
                for k in range(self.num_of_agents):
                    ten[k]['loc'], ten[k]['val'] = agentlist[k].solve_coll(ten_loc, vals)
=======
                if location in [d['loc'] for d in curr]:
                    if ten[[d['loc'] for d in curr].index(location)]['loc'] == curr[[d['loc'] for d in ten].index(location)]['loc']:
                        print([d['loc'] for d in curr].index(location),[d['loc'] for d in ten].index(location))
                        if [d['loc'] for d in curr].index(location) != [d['loc'] for d in ten].index(location):
                            coll = True
                            break

            while coll:
                for k in range(self.num_of_agents):
                    ten = agentlist[k].solve_coll(curr, ten)

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
                                print([d['loc'] for d in curr].index(location), [d['loc'] for d in ten].index(location))
                                if [d['loc'] for d in curr].index(location) != [d['loc'] for d in ten].index(location):
                                    coll = True
                                    break


            for l in range(self.num_of_agents):
                moves[l].append(ten[l]['loc'])
                agentlist[l].update_loc(ten[l]['loc'])

            vals = [d['val'] for d in ten]
>>>>>>> main


        result = moves

<<<<<<< HEAD
        
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)
=======
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(
            get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)

        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)
>>>>>>> main
