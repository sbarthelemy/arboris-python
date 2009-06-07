# coding=utf-8
"""
Miscaleneous classes

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

import numpy
from numpy import empty

DTYPE = numpy.float
cdef asym(DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z):
    """
    Asymetric matrix (in so(3) )from a vector (in R^3).

    Example:

    >>> asym(1., 2., 3.)
    array([[ 0., -3.,  2.],
           [ 3.,  0., -1.],
           [-2.,  1.,  0.]])

    """
    cdef numpy.ndarray[DTYPE_t, ndim=2] R = empty((3,3), dtype=DTYPE)
    R[0,0] = 0.
    R[0,1] = -v_z 
    R[0,2] = v_y
    R[1,0] = v_z
    R[1,1] = 0.
    R[1,2] = -v_x
    R[2,0] = -v_y
    R[2,1] = v_x
    R[2,2] = 0
    return R



#import Exception 
#cimport Exception

#cdef class NamedObject(object):
    #    """
    #A class for anything named to depend from.
    #"""

#    def __init__(self, name=None):
    #        self.name = name

#def class DuplicateNameError(Exception):    #can't declare this one cdef 'cause Exception is not an extended object...
#    pass                                    #must see an equivalent in cython

