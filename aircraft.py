"""HI"""
from single_agent_planner import compute_heuristics


class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.loc = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = heuristics

    def calc_next(self):
        if self.loc == self.goal:
            return self.loc, 0

        moves = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        next_move = self.loc
        for move in moves:
            possible_move = (self.loc[0] + move[0], self.loc[1] + move[1])
            try:
                if self.heuristics[possible_move] < self.heuristics[next_move]:
                    next_move = possible_move
            except KeyError:
                continue

        return next_move, self.heuristics[next_move], self.loc

    def update_loc(self, loc):
        self.loc = loc

    def solve_coll(self, ten):

        pass
