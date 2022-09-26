# -*- coding: utf-8 -*-
class Node(object):
    """
    basic node, saves X, Y and Z coordinates on some grid and determine if
    it is walkable.
    """
    def __init__(self, x=0, y=0, z=0, walkable=True, weight=1):
        # Coordinates
        self.x = x
        self.y = y
        self.z = z

        # Whether this node can be walked through.
        self.walkable = walkable

        # used for weighted algorithms
        self.weight = weight

        # values used in the finder
        self.cleanup()

    @classmethod
    def from_array(cls, array, walkable=True, weight=1):
        """
        create a node from an array
        """
        return cls(array[0], array[1], array[2], walkable, weight)

    def __lt__(self, other):
        """
        nodes are sorted by f value (see a_star.py)

        :param other: compare Node
        :return:
        """
        return self.f < other.f

    def cleanup(self):
        """
        reset all calculated values, fresh start for pathfinding
        """
        # cost from this node to the goal (for A* including the heuristic)
        self.h = 0.0

        # cost from the start node to this node 
        # (calculated by distance function, e.g. including diagonal movement)
        self.g = 0.0

        # overall cost for a path using this node (f = g + h )
        self.f = 0.0

        self.opened = 0
        self.closed = False

        # used for backtracking to the start point
        self.parent = None

        # used for recurion tracking of IDA*
        self.retain_count = 0
        # used for IDA* and Jump-Point-Search
        self.tested = False
