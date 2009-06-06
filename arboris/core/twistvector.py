# coding=utf-8
"""
Functions for working with twists stored as [w,v]
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from numpy import array, sin, cos, eye, dot, hstack, vstack
from numpy.linalg import norm

def adjacency(tw):
    """
    return the adjacency matrix
    """
    return array(
        [[     0,-tw[2], tw[1],      0,     0,     0],
         [ tw[2],     0,-tw[0],      0,     0,     0],
         [-tw[1], tw[0],     0,      0,     0,     0],
         [     0,-tw[5], tw[4],      0,-tw[2], tw[1]],
         [ tw[5],     0,-tw[3],  tw[2],     0,-tw[0]],
         [-tw[4], tw[3],     0, -tw[1], tw[0],     0]])

def exp(tw):
    """
    return the exponential of the twist matrix
    """
    w = tw[0:3]
    v = tw[3:6]
    wx = array(
        [[     0,-tw[2], tw[1]],
         [ tw[2],     0,-tw[0]],
         [-tw[1], tw[0],     0]])
    t = norm(w)
    if t >= 0.001:
        cc = (1-cos(t))/t**2
        sc = sin(t)/t
        dsc = (t-sin(t))/t**3
    else:
        cc = 1./2.
        sc = 1.-t**2/6.
        dsc = 1./6.
    
    R = eye(3) + sc*wx + cc*dot(wx,wx)
    p = dot(sc*eye(3)+cc*wx+dsc*dot(w,w.T), v)
    return vstack((hstack((R, p.reshape(3,1))),
                   array([[0., 0., 0., 1.]])))
