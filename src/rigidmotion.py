# coding=utf-8
"""
TODO: add doc here
TODO: add doctests
TODO: add support for FreeMotion
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")
import numpy as np
import homogeneousmatrix as Hg
import twistvector as T
from abc import ABCMeta, abstractmethod

class RigidMotion(object):

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

    @abstractmethod
    def pose(self):
        """Return the pose as an homogeneous matrix. 
        """
        pass

    def ipose(self):
        """Inverse of pose()
        """
        return Hg.inv(self.pose())

    @abstractmethod
    def twist(self):
        """Return the velocity as a twist vector. 
        """
        pass

    def itwist(self):
        return -np.dot(np.asarray(self.iadjoint()),self.twist())

    def adjoint(self):
        return Hg.adjoint(self.pose())

    def iadjoint(self):
        return Hg.adjoint(self.ipose())

    def adjacency(self):
        return T.adjacency(self.twist())

    def iadjacency(self):
        return T.adjacency(self.itwist())

    def dadjoint(self):
        return np.dot(np.asarray(self.adjoint()),self.adjacency())
    
    def idadjoint(self):
        return np.dot(np.asarray(self.iadjoint()),self.iadjacency())


