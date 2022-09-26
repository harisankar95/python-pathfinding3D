# -*- coding: utf-8 -*-
import math
import copy


# square root of 2 and 3 for diagonal distance
SQRT2 = math.sqrt(2)
SQRT3 = math.sqrt(3)


def backtrace(node):
    """
    Backtrace according to the parent records and return the path.
    (including both start and end nodes)
    """
    path = [(node.x, node.y, node.z)]
    while node.parent:
        node = node.parent
        path.append((node.x, node.y, node.z))
    path.reverse()
    return path


def bi_backtrace(node_a, node_b):
    """
    Backtrace from start and end node, returns the path for bi-directional A*
    (including both start and end nodes)
    """
    path_a = backtrace(node_a)
    path_b = backtrace(node_b)
    path_b.reverse()
    return path_a + path_b