# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")
from numpy import dot 
import arboris.homogeneousmatrix as Hg
import arboris.twistvector as T
from abc import ABCMeta, abstractproperty

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

    @abstractproperty
    def pose(self):
        """Return the pose as an homogeneous matrix. 
        """
        pass

    @property
    def ipose(self):
        """Inverse of pose
        """
        return Hg.inv(self.pose)

    @abstractproperty
    def twist(self):
        """Return the velocity as a twist vector. 
        """
        pass

    @property
    def itwist(self):
        return -dot(self.iadjoint, self.twist)

    @property
    def adjoint(self):
        return Hg.adjoint(self.pose)

    @property
    def iadjoint(self):
        return Hg.adjoint(self.ipose)

    @property
    def adjacency(self):
        return T.adjacency(self.twist)

    @property
    def iadjacency(self):
        return T.adjacency(self.itwist)

    @property
    def dadjoint(self):
        return dot(self.adjoint, self.adjacency)
    
    @property
    def idadjoint(self):
        return dot(self.iadjoint, self.iadjacency)


