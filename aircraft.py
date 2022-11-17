"""HI"""
import sys

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

    def calc_next(self, curr, collision_locs=None):
        if collision_locs is None:
            collision_locs = []
        moves = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        next_move = self.loc
        for move in moves:
            possible_move = (self.loc[0] + move[0], self.loc[1] + move[1])
            try:
                if self.my_map[possible_move[0]][possible_move[1]]:
                    continue
            except IndexError:
                continue
            try:
                next_locs = collision_locs[:]
                next_locs.insert(self.id, (-1, -1))
                prev_locs = [d['loc'] for d in curr]
                prev_locs[self.id] = (-1, -1)
                if (possible_move in prev_locs) and (self.loc in next_locs):
                    if prev_locs.index(possible_move) == next_locs.index(self.loc):
                        continue
            except KeyError:
                pass

            adjust = 0
            try:
                if next_move in collision_locs:
                    adjust = 1e3
                if (self.heuristics[possible_move] < self.heuristics[next_move] + adjust) and not (
                        possible_move in collision_locs):
                    next_move = possible_move

            except KeyError:
                continue

        return next_move, self.heuristics[next_move], self.loc

    def update_loc(self, loc):
        self.loc = loc

    def solve_coll(self, curr, ten):
        next_loc = ten[self.id]['loc']
        other_locs = [d['loc'] for d in ten]
        if (next_loc in [d['loc'] for d in curr]) and (curr[self.id]['loc'] in [d['loc'] for d in ten]):
            if [d['loc'] for d in ten].index(curr[self.id]['loc']) != self.id:
                ten[self.id]['loc'], ten[self.id]['val'], _ = self.calc_next(curr, other_locs)
                return ten

        if next_loc in other_locs:
            indexes = [i for i, x in enumerate(other_locs) if x == next_loc]
            indexes.remove(self.id)
            del other_locs[self.id]
            if all(v >= ten[self.id]['val'] for v in [ten[i]['val'] for i in indexes]):
                ten[self.id]['loc'], ten[self.id]['val'], _ = self.calc_next(curr, other_locs)
                return ten

        return ten
