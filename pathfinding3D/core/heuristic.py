# -*- coding: utf-8 -*-
import math
from .util import SQRT2, SQRT3


def null(dx, dy, dz):
    """
    special heuristic for Dijkstra
    return 0, so node.h will always be calculated as 0,
    distance cost (node.f) is calculated only from
    start to current point (node.g)
    """
    return 0


def manhattan(dx, dy, dz):
    """manhattan heuristics"""
    return dx + dy + dz


def euclidean(dx, dy, dz):
    """euclidean distance heuristics"""
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def chebyshev(dx, dy, dz):
    """ Chebyshev distance. """
    return max(dx, dy, dz)


def octile(dx, dy, dz):
    """ Octile distance. """
    dmax = max(dx, dy, dz)
    dmin = min(dx, dy, dz)
    dmid = dx + dy + dz - dmax - dmin

    return dmax + (SQRT2 - 1) * dmid + (SQRT3 - SQRT2) * dmin