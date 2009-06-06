# coding=utf-8
"""
TODO: add doc here
TODO: add doctests
TODO: add support for FreeMotion
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

cimport numpy as np
cimport homogeneousmatrix as Hg
cimport twistvector as T

cdef class RigidMotion(object):
    cdef pose(self):
    cdef ipose(self):
    cdef twist(self):
    cdef itwist(self):
    cdef adjoint(self):
    cdef iadjont(self):
    cdef adjacency(self):
    cdef iadjacency(self):
    cdef dadjoint(self):
    cdef idadjoint(self):

