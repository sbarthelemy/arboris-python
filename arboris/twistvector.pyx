# coding=utf-8
"""
Functions for working with twists stored as [w,v]
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from numpy import zeros, empty, eye, dot
from misc_c cimport asym, sin, cos, sqrt
cimport numpy as np
import numpy as np
DTYPE = np.float 
ctypedef np.float_t DTYPE_t

cpdef adjacency(np.ndarray tw):
    """
    Adjacency matrix of the twist.

    Example:
        
    >>> from numpy import array
    >>> t = array([1., 2., 3., 10., 11., 12.])
    >>> adjacency(t)
    array([[  0.,  -3.,   2.,   0.,   0.,   0.],
           [  3.,   0.,  -1.,   0.,   0.,   0.],
           [ -2.,   1.,   0.,   0.,   0.,   0.],
           [  0., -12.,  11.,   0.,  -3.,   2.],
           [ 12.,   0., -10.,   3.,   0.,  -1.],
           [-11.,  10.,   0.,  -2.,   1.,   0.]])

    """
    cdef np.ndarray[DTYPE_t, ndim=2] A = empty((6,6),dtype=DTYPE)
    A[0,0] = 0.
    A[0,1] = -tw[2]
    A[0,2] = tw[1]
    A[0,3] = 0. 
    A[0,4] = 0. 
    A[0,5] = 0. 

    A[1,0] = tw[2]
    A[1,1] = 0. 
    A[1,2] = -tw[0]
    A[1,3] = 0. 
    A[1,4] = 0. 
    A[1,5] = 0. 

    A[2,0] = -tw[1]
    A[2,1] = tw[0]
    A[2,2] = 0. 
    A[2,3] = 0. 
    A[2,4] = 0. 
    A[2,5] = 0. 

    A[3,0] = 0. 
    A[3,1] = -tw[5]
    A[3,2] = tw[4]
    A[3,3] = 0. 
    A[3,4] = -tw[2]
    A[3,5] = tw[1]

    A[4,0] = tw[5]
    A[4,1] = 0.
    A[4,2] = -tw[3]
    A[4,3] = tw[2]
    A[4,4] = 0.
    A[4,5] = -tw[0]

    A[5,0] = -tw[4]
    A[5,1] = tw[3]
    A[5,2] = 0.
    A[5,3] = -tw[1]
    A[5,4] = tw[0]
    A[5,5] = 0.
    return A

cpdef exp(np.ndarray tw):
    """
    Exponential (homogeneous matrix) of the twist.

    Example:
    
    >>> from numpy import array
    >>> t = array([1., 2., 3., 10., 11., 12.])
    >>> exp(t)
    array([[ -0.69492056,   0.71352099,   0.08929286,  35.29778399],
           [ -0.19200697,  -0.30378504,   0.93319235,  38.66708228],
           [  0.69297817,   0.6313497 ,   0.34810748,  34.99594133],
           [  0.        ,   0.        ,   0.        ,   1.        ]])

    """
    cdef double cc, sc, dsc
    cdef double t = sqrt(tw[0]*tw[0] + tw[1]*tw[1] + tw[2]*tw[2])
    cdef np.ndarray[DTYPE_t, ndim=2] H = empty((4,4),dtype=DTYPE)

    if t >= 0.001:
        cc = (1-cos(t))/(t**2)
        sc = sin(t)/t
        dsc = (t-sin(t))/(t**3)
    else:
        cc = 1./2.
        sc = 1.-t**2/6.
        dsc = 1./6.0
    cdef np.ndarray[DTYPE_t, ndim=2] wx = asym(tw[0], tw[1], tw[2])
    H[0:3, 0:3] = eye(3) + sc*wx + cc*dot(wx,wx)
    H[0:3,3] = dot(sc*eye(3) + cc*wx + dsc*dot(tw[0:3],tw[0:3].T), 
                   tw[3:6])
    H[3,0] = 0.
    H[3,1] = 0.
    H[3,2] = 0.
    H[3,3] = 1.
    return H
