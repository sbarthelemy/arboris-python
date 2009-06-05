# coding=utf-8
"""
TODO: add doc here
TODO: add doctests
TODO: add support for FreeMotion
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

#a verifier s'il faut faire a chaque fois import et cimport
import numpy as np
cimport numpy as np
import homogeneousmatrix as Hg
cimport homogeneousmatrix_c as Hg
import twistvector as T
cimport twistvector_c as T
from abc import ABCMeta, abstractproperty
#from abc cimport ABCMeta #, abstractproperty: no such class for cython modules yet...look for how to substituat it

cdef class RigidMotion(object):

    """Model a rigid motion, including relative pose and velocity

    n: new frame
    r: reference frame
    self.pose(): H_rn
    self.ipose(): H_nr
    self.twist(): T_nr
    self.itwist(): T_rn
    self.adjoint(): Ad_rn == adjoint(H_rn)
    self.iadjoint(): Ad_nr == adjoint(H_nr)
    self.adjacency(): adjacency(T_nr)
    self.iadjacency(): adjacency(T_rn)
    self.dadjoint(): dAd_rn
    self.idadjoint():dAd_nr
    """
    __metaclass__ = ABCMeta

#    abstractproperty pose:          #@abstractproperty: verifier s'il existe un equivalent en cython....
#        def __get__(self):
#        """Return the pose as an homogeneous matrix. 
#        """
#            pass

    property pose:          
        """Return the pose as an homogeneous matrix. 
        """
        def __get__(self):
            pass
    property ipose:
        """Inverse of pose()
        """
        def __get__(self):
            return Hg.inv(self.pose)

#    abstractproperty twist:
#        def __get__(self):
#        """Return the velocity as a twist vector. 
#        """
#            pass

    property twist:
        """Return the velocity as a twist vector. 
        """
        def __get__(self):
            pass

    property itwist:
        def __get__(self):
            return -np.dot(self.iadjoint, self.twist)  

    property adjoint:
        def __get__(self):
            return Hg.adjoint(self.pose)  

    property iadjoint:
        def __get__(self):
            return Hg.adjoint(self.ipose)

    property adjacency:
        def __get__(self):
            return T.adjacency(self.twist)

    property iadjacency:
        def __get__(self):
            return T.adjacency(self.itwist)

    property dadjoint:
        def __get__(self):
            return np.dot(np.asarray(self.adjoint),self.adjacency)
    
    property idadjoint:
        def __get__(self):
            return np.dot(np.asarray(self.iadjoint),self.iadjacency)


