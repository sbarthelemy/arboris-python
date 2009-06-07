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

    Example:

    >>> t = array([1., 2., 3., 10., 11., 12.])
    >>> adjacency(t)
    array([[  0.,  -3.,   2.,   0.,   0.,   0.],
           [  3.,   0.,  -1.,   0.,   0.,   0.],
           [ -2.,   1.,   0.,   0.,   0.,   0.],
           [  0., -12.,  11.,   0.,  -3.,   2.],
           [ 12.,   0., -10.,   3.,   0.,  -1.],
           [-11.,  10.,   0.,  -2.,   1.,   0.]])

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

    Example:

    >>> t = array([1., 2., 3., 10., 11., 12.])
    >>> exp(t)
    array([[ -0.69492056,   0.71352099,   0.08929286,  35.29778399],
           [ -0.19200697,  -0.30378504,   0.93319235,  38.66708228],
           [  0.69297817,   0.6313497 ,   0.34810748,  34.99594133],
           [  0.        ,   0.        ,   0.        ,   1.        ]])

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
