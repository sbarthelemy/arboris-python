# coding=utf-8
"""
Functions for working with twists stored as [w,v]
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

import numpy as np

def adjacency(tw):
    """
    return the adjacency matrix
    """
    return np.array(
        [[     0,-tw[2], tw[1],      0,     0,     0],
         [ tw[2],     0,-tw[0],      0,     0,     0],
         [-tw[1], tw[0],     0,      0,     0,     0],
         [     0,-tw[5], tw[4],      0,-tw[2], tw[1]],
         [ tw[5],     0,-tw[3],  tw[2],     0,-tw[0]],
         [-tw[4], tw[3],     0, -tw[1], tw[0],     0]])
