import sys

from single_agent_planner import compute_heuristics
import numpy as np
import random


class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id, prio):
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
        self.priority = prio

    def calc_next(self, curr, look_steps, collision_locs=None):
        if collision_locs is None:
            collision_locs = []
        moves = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        #print(look_steps)
        forward_value = 0
        old_forward = 0
        next_move = self.loc
        old_forward = forward(self, next_move, look_steps)

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
                forward_value = forward(self, possible_move, look_steps)
                #print(self.heuristics[possible_move], forward_value,self.heuristics[next_move], old_forward)
                if (self.heuristics[possible_move] + forward_value < self.heuristics[next_move] + old_forward + adjust) and not (
                        possible_move in collision_locs):
                    next_move = possible_move
                    old_forward = forward_value
                # elif (self.heuristics[possible_move] + forward_value == self.heuristics[next_move] + old_forward + adjust) and not (
                #        possible_move in collision_locs):
                #    if bool(random.getrandbits(1)):
                #        next_move = possible_move
                #        old_forward = forward_value


            except KeyError:
                continue

        return next_move, self.heuristics[next_move], self.loc

    def update_loc(self, loc):
        self.loc = loc

    def solve_coll(self, curr, ten, look_steps):
        next_loc = ten[self.id]['loc']
        other_locs = [d['loc'] for d in ten]
        if (next_loc in [d['loc'] for d in curr]) and (curr[self.id]['loc'] in [d['loc'] for d in ten]):
            if [d['loc'] for d in ten].index(curr[self.id]['loc']) != self.id:
                ten[self.id]['loc'], ten[self.id]['val'], _ = self.calc_next(curr, look_steps, other_locs)
                return ten

        if next_loc in other_locs:
            indexes = [i for i, x in enumerate(other_locs) if x == next_loc]
            indexes.remove(self.id)
            del other_locs[self.id]
            if all(v >= ten[self.id]['val'] for v in [ten[i]['val'] for i in indexes]):
                ten[self.id]['loc'], ten[self.id]['val'], _ = self.calc_next(curr, look_steps, other_locs)
                #self.priority = True
                return ten
            elif self.priority:
                ten[self.id]['loc'], ten[self.id]['val'], _ = self.calc_next(curr, look_steps, other_locs)
                self.priority = False
                return ten


        return ten


def forward(self, loc, look_steps):
    pos = loc[:]
    ret = 0
    add = 0
    for i in range(look_steps):
        add, pos = forward_look(self, pos)
        ret += add
    return ret


def forward_look(self, loc):
    moves = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    d = []
    l = []
    for move in moves:
        try:
            if self.my_map[move[0]][move[1]]:
                continue
            new = (loc[0] + move[0], loc[1] + move[1])
            d.append(self.heuristics[new])
            l.append(new)

        except KeyError:
            continue

    index_min = np.argmin(d)
    return d[index_min], l[index_min]