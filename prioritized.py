import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        timeout = timer.time() + 1
        longest_path = 50
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints, longest_path)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            # print(path)
            
            for j in range(len(path)):
                for k in range(self.num_of_agents):
                    if i == k: 
                        continue
                    con_loc = path[j]


                    # constraints.append({'agent': k, 'loc': [con_loc], 't_step': j + 1})
                    # constraints.append({'agent': k, 'loc': [con_loc], 't_step': j + 2})

                    if len(path) > longest_path:
                        longest_path = len(path)
                    elif j == len(path) - 1:
                        for m in range(j + 1, longest_path):
                            # print(k, con_loc, m)
                            constraints.append({'agent': k, 'loc': [con_loc], 't_step': m})
                    
                    if j < len(path) - 1:
                        con_loc2 = path[j + 1]
                        # constraints.append({'agent': k, 'loc': [con_loc], 't_step': j + 2})
                        constraints.append({'agent': k, 'loc': [con_loc], 't_step': j + 1})
                        constraints.append({'agent': k, 'loc': [con_loc2], 't_step': j + 1})
                        constraints.append({'agent': k, 'loc': [con_loc, con_loc2], 't_step': j+1})
                        # print('1', k, con_loc, j + 1)
                        # print('2', k, con_loc2, j + 1)
                        print('test' , k, [con_loc, con_loc2], j+1) 

            ##############################

        self.CPU_time = timer.time() - start_time
        if  timer.time() > timeout:
            print(start_time)
            print(timer.time())
            print('No result')
            return None, self.CPU_time
             
      

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result, self.CPU_time
