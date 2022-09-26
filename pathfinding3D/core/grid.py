# -*- coding: utf-8 -*-
from .node import Node
try:
    import numpy as np
    USE_NUMPY = True
except ImportError:
    USE_NUMPY = False
from .diagonal_movement import DiagonalMovement


def build_nodes(width, height, depth, matrix=None, inverse=False):
    """
    create nodes according to grid size. If a matrix is given it
    will be used to determine what nodes are walkable.
    :rtype : list
    """
    nodes = []
    use_matrix = (isinstance(matrix, (tuple, list))) or \
        (USE_NUMPY and isinstance(matrix, np.ndarray) and matrix.size > 0)

    for x in range(width):
        nodes.append([])
        for y in range(height):
            nodes[x].append([])
            for z in range(depth):
                # 1, '1', True will be walkable
                # while others will be obstacles
                # if inverse is False, otherwise
                # it changes
                weight = int(matrix[x][y][z]) if use_matrix else 1
                walkable = weight <= 0 if inverse else weight >= 1

                nodes[x][y].append(Node(x=x, y=y, z=z, walkable=walkable, weight=weight))
    return nodes


class Grid(object):
    def __init__(self, width=0, height=0, depth=0, matrix=None, inverse=False):
        """
        a grid represents the map (as 3d-list of nodes).
        """
        self.width = width
        self.height = height
        self.depth = depth
        if isinstance(matrix, (tuple, list)) or (
                USE_NUMPY and isinstance(matrix, np.ndarray) and
                matrix.size > 0):
            self.width = len(matrix)
            self.height = self.height = len(matrix[0]) if self.width > 0 else 0
            self.depth = self.depth = len(matrix[0][0]) if self.height > 0 else 0
        if self.width > 0 and self.height > 0 and self.depth > 0:
            self.nodes = build_nodes(self.width, self.height, self.depth, matrix, inverse)
        else:
            self.nodes = [[[]]]

    def node(self, x, y, z):
        """
        get node at position
        :param x: x pos
        :param y: y pos
        :param z: z pos
        :return:
        """
        return self.nodes[x][y][z]

    def node_from_array(self, array):
        """
        create a node from an array
        :param array: [x, y, z]
        """
        return self.node(array[0], array[1], array[2])
    
    def inside(self, x, y, z):
        """
        check, if field position is inside map
        :param x: x pos
        :param y: y pos
        :param z: z pos
        :return:
        """
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth

    def walkable(self, x, y, z):
        """
        check, if the tile is inside grid and if it is set as walkable
        """
        return self.inside(x, y, z) and self.nodes[x][y][z].walkable

    def neighbors(self, node, diagonal_movement=DiagonalMovement.never):
        """
        get all neighbors of one node
        :param node: node
        """
        x = node.x
        y = node.y
        z = node.z
        neighbors = []
        # current plane
        s0 = d0 = s1 = d1 = s2 = d2 = s3 = d3 = False
        # upper plane
        ts0 = td0 = ts1 = td1 = ts2 = td2 = ts3 = td3 = tt = False
        # lower plane
        ds0 = dd0 = ds1 = dd1 = ds2 = dd2 = ds3 = dd3 = dd = False

        # ↑
        if self.walkable(x, y + 1, z):
            neighbors.append(self.nodes[x][y + 1][z])
            s0 = True
        # →
        if self.walkable(x + 1, y, z):
            neighbors.append(self.nodes[x + 1][y][z])
            s1 = True
        # ↓
        if self.walkable(x, y - 1, z):
            neighbors.append(self.nodes[x][y - 1][z])
            s2 = True
        # ←
        if self.walkable(x - 1, y, z):
            neighbors.append(self.nodes[x - 1][y][z])
            s3 = True
        # tt
        if self.walkable(x, y, z + 1):
            neighbors.append(self.nodes[x][y][z + 1])
            tt = True
        # dd
        if self.walkable(x, y, z - 1):
            neighbors.append(self.nodes[x][y][z - 1])
            dd = True

        if diagonal_movement == DiagonalMovement.never:
            return neighbors

        if diagonal_movement == DiagonalMovement.only_when_no_obstacle:
            d0 = s3 and s0
            d1 = s0 and s1
            d2 = s1 and s2
            d3 = s2 and s3

            ts0 = tt and s0
            ts1 = tt and s1
            ts2 = tt and s2
            ts3 = tt and s3

            ds0 = dd and s0
            ds1 = dd and s1
            ds2 = dd and s2
            ds3 = dd and s3

        elif diagonal_movement == DiagonalMovement.if_at_most_one_obstacle:
            d0 = s3 or s0
            d1 = s0 or s1
            d2 = s1 or s2
            d3 = s2 or s3

            ts0 = tt or s0
            ts1 = tt or s1
            ts2 = tt or s2
            ts3 = tt or s3

            ds0 = dd or s0
            ds1 = dd or s1
            ds2 = dd or s2
            ds3 = dd or s3

        elif diagonal_movement == DiagonalMovement.always:
            d0 = d1 = d2 = d3 = True
            ts0 = ts1 = ts2 = ts3 = True
            ds0 = ds1 = ds2 = ds3 = True

        # ↖
        if d0 and self.walkable(x - 1, y + 1, z):
            neighbors.append(self.nodes[x - 1][y + 1][z])
        else:
            d0 = False

        # ↗
        if d1 and self.walkable(x + 1, y + 1, z):
            neighbors.append(self.nodes[x + 1][y + 1][z])
        else:
            d1 = False

        # ↘
        if d2 and self.walkable(x + 1, y - 1, z):
            neighbors.append(self.nodes[x + 1][y - 1][z])
        else:
            d2 = False

        # ↙
        if d3 and self.walkable(x - 1, y - 1, z):
            neighbors.append(self.nodes[x - 1][y - 1][z])
        else:
            d3 = False

        # t↑
        if ts0 and self.walkable(x, y + 1, z + 1):
            neighbors.append(self.nodes[x][y + 1][z + 1])
        else:
            ts0 = False

        # t→
        if ts1 and self.walkable(x + 1, y, z + 1):
            neighbors.append(self.nodes[x + 1][y][z + 1])
        else:
            ts1 = False

        # t↓
        if ts2 and self.walkable(x, y - 1, z + 1):
            neighbors.append(self.nodes[x][y - 1][z + 1])
        else:
            ts2 = False

        # t←
        if ts3 and self.walkable(x - 1, y, z + 1):
            neighbors.append(self.nodes[x - 1][y][z + 1])
        else:
            ts3 = False

        # d↑
        if ds0 and self.walkable(x, y + 1, z - 1):
            neighbors.append(self.nodes[x][y + 1][z - 1])
        else:
            ds0 = False

        # d→
        if ds1 and self.walkable(x + 1, y, z - 1):
            neighbors.append(self.nodes[x + 1][y][z - 1])
        else:
            ds1 = False

        # d↓
        if ds2 and self.walkable(x, y - 1, z - 1):
            neighbors.append(self.nodes[x][y - 1][z - 1])
        else:
            ds2 = False
        
        # d←
        if ds3 and self.walkable(x - 1, y, z - 1):
            neighbors.append(self.nodes[x - 1][y][z - 1])
        else:
            ds3 = False

        # remaining upper and lower diagonals
        if diagonal_movement == DiagonalMovement.only_when_no_obstacle:
            td0 = d0 and s0 and s3 and tt and ts0 and ts3
            td1 = d1 and s0 and s1 and tt and ts0 and ts1
            td2 = d2 and s1 and s2 and tt and ts1 and ts2
            td3 = d3 and s2 and s3 and tt and ts2 and ts3

            dd0 = d0 and s0 and s3 and dd and ds0 and ds3
            dd1 = d1 and s0 and s1 and dd and ds0 and ds1
            dd2 = d2 and s1 and s2 and dd and ds1 and ds2
            dd3 = d3 and s2 and s3 and dd and ds2 and ds3

        elif diagonal_movement == DiagonalMovement.if_at_most_one_obstacle:
            td0 = sum([d0, s0, s3, tt, ts0, ts3]) >= 5
            td1 = sum([d1, s0, s1, tt, ts0, ts1]) >= 5
            td2 = sum([d2, s1, s2, tt, ts1, ts2]) >= 5
            td3 = sum([d3, s2, s3, tt, ts2, ts3]) >= 5

            dd0 = sum([d0, s0, s3, dd, ds0, ds3]) >= 5
            dd1 = sum([d1, s0, s1, dd, ds0, ds1]) >= 5
            dd2 = sum([d2, s1, s2, dd, ds1, ds2]) >= 5
            dd3 = sum([d3, s2, s3, dd, ds2, ds3]) >= 5

        elif diagonal_movement == DiagonalMovement.always:
            td0 = td1 = td2 = td3 = True
            dd0 = dd1 = dd2 = dd3 = True
        
        # t↖
        if td0 and self.walkable(x - 1, y + 1, z + 1):
            neighbors.append(self.nodes[x - 1][y + 1][z + 1])

        # t↗
        if td1 and self.walkable(x + 1, y + 1, z + 1):
            neighbors.append(self.nodes[x + 1][y + 1][z + 1])

        # t↘
        if td2 and self.walkable(x + 1, y - 1, z + 1):
            neighbors.append(self.nodes[x + 1][y - 1][z + 1])

        # t↙
        if td3 and self.walkable(x - 1, y - 1, z + 1):
            neighbors.append(self.nodes[x - 1][y - 1][z + 1])

        # d↖
        if dd0 and self.walkable(x - 1, y + 1, z - 1):
            neighbors.append(self.nodes[x - 1][y + 1][z - 1])

        # d↗
        if dd1 and self.walkable(x + 1, y + 1, z - 1):
            neighbors.append(self.nodes[x + 1][y + 1][z - 1])

        # d↘
        if dd2 and self.walkable(x + 1, y - 1, z - 1):
            neighbors.append(self.nodes[x + 1][y - 1][z - 1])

        # d↙
        if dd3 and self.walkable(x - 1, y - 1, z - 1):
            neighbors.append(self.nodes[x - 1][y - 1][z - 1])
        
        return neighbors

    def cleanup(self):
        for x_nodes in self.nodes:
            for y_nodes in x_nodes:
                for node in y_nodes:
                    node.cleanup()
