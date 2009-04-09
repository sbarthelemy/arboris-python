# coding=utf-8
"""
TODO: add doc here
TODO: add doctests
TODO: add support for FreeMotion
TODO: rename HingeJoint to RzJoint ?
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


class Joint(RigidMotion):
    """any joint
    H
    T_
    """    
    __metaclass__ = ABCMeta

    def twist(self):
        return np.dot(self.jacobian(),self.gvel)


    @abstractmethod
    def ndof(self):
        """Number of degrees of freedom of the joint
        """
        pass

    @abstractmethod
    def jacobian(self):
        pass

    @abstractmethod
    def djacobian(self):
        pass


class FreeJoint(Joint):

    """Free joint (6-dof)
    """
    def __init__(self, gpos=None, gvel=None):
        """
        example:
        >>> j = FreeJoint()
        >>> j.gpos
        array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]])
        >>> j.gvel
        array([ 0.,  0.,  0.,  0.,  0.,  0.])
        """
        if gpos == None:
            gpos = np.eye(4)
        if gvel == None:
            gvel = np.zeros((6))
        self.gpos = np.array(gpos).reshape((4,4))
        self.gvel = np.array(gvel).reshape((6))

    def ndof(self):
        return 6
    
    def pose(self):
        return self.gpos.copy()

    def twist(self):
        return self.gvel.copy()

    def jacobian(self):
        return np.eye(6)

    def djacobian(self):
        return np.zeros((6,6))

#class PivotJoint(Joint):
#
#    """Pivot (2-dof)
#    """
#
#class BallJoint(Joint):
#
#    """Ball and socket (3-dof)
#    """

class RzRyRxJoint(Joint):
    """Ball and socket (3-dof) implemented with 3 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry*Rx
    """
    def __init__(self, gpos=[0.,0.,0.], gvel=[0.,0.,0.]):
        self.gpos = np.array(gpos).reshape((3))
        self.gvel = np.array(gvel).reshape((3))

    def ndof(self):
        return 3

    def pose(self):
        return Hg.rotzyx(self.gpos)

    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = np.sin(self.gpos[0])
        cx = np.cos(self.gpos[0])
        sy = np.sin(self.gpos[1])
        cy = np.cos(self.gpos[1])
        return np.array(
            [[ 1.   ,  0. , -sy    ],
             [ 0.   , cx  ,  sx*sy ],
             [ 0.   ,-sx  ,  cx*cy ]])
    
    def djacobian(self):
        sx = np.sin(self.gpos[0])
        cx = np.cos(self.gpos[0])
        sy = np.sin(self.gpos[1])
        cy = np.cos(self.gpos[1])
        dx = self.gvel[0]
        dy = self.gvel[1]
        return np.array(
            [[ 0.   , 0.    ,-dy*cy             ],
             [ 0.   ,-dx*sx , dx*cx*sy+dy*sx*cy ],
             [ 0.   ,-dx*cx ,-dx*sx*cy-dy*cx*sy ]])


class RzRyJoint(Joint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.]):
        self.gpos = np.array(gpos).reshape((2))
        self.gvel = np.array(gvel).reshape((2))

    def ndof(self):
        return 2

    def pose(self):
        return Hg.rotzy(self.gpos)

    def jacobian(self):
        """
        T_n/r = 
        """   
        sy = np.sin(self.gpos[0])
        cy = np.cos(self.gpos[0])
        return np.array(
            [[ 0.   , -sy ],
             [ 1.   ,  0. ],
             [ 0.   ,  cy ]])
    
    def djacobian(self):
        sy = np.sin(self.gpos[0])
        cy = np.cos(self.gpos[0])
        dy = self.gvel[0]
        return np.array(
            [[ 0.   ,-dy*cy ],
             [ 0.   , 0.    ],
             [ 0.   ,-dy*sy ]])
             
             
class RzRxJoint(Joint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.]):
        self.gpos = np.array(gpos).reshape((2))
        self.gvel = np.array(gvel).reshape((2))

    def ndof(self):
        return 2

    def pose(self):
        return Hg.rotzx(self.gpos)

    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = np.sin(self.gpos[0])
        cx = np.cos(self.gpos[0])
        return np.array(
            [[ 1.   ,  0. ],
             [ 0.   ,  sx ],
             [ 0.   ,  cx ]])
    
    def djacobian(self):
        sy = np.sin(self.gpos[0])
        cy = np.cos(self.gpos[0])
        dy = self.gvel[0]
        return np.array(
            [[ 0.   , 0.    ],
             [ 0.   , dx*cx ],
             [ 0.   ,-dx*sx ]])
             
             
class RyRxJoint(Joint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.]):
        self.gpos = np.array(gpos).reshape((2))
        self.gvel = np.array(gvel).reshape((2))

    def ndof(self):
        return 2

    def pose(self):
        return Hg.rotyx(self.gpos)

    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = np.sin(self.gpos[0])
        cx = np.cos(self.gpos[0])
        return np.array(
            [[ 1.   ,  0. ],
             [ 0.   ,  cx ],
             [ 0.   , -sx ]])
    
    def djacobian(self):
        sy = np.sin(self.gpos[0])
        cy = np.cos(self.gpos[0])
        dy = self.gvel[0]
        return np.array(
            [[ 0.   , 0.    ],
             [ 0.   ,-dx*sx ],
             [ 0.   ,-dx*cx ]])
             
             
class HingeJoint(Joint):

    """Hinge (1-dof) with axis in the z-direction
    """
    def __init__(self, gpos=0., gvel=0.):
        """
        example:
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.gpos
        array([ 1.57])
        >>> j.gvel
        array([ 1.])
        """
        self.gpos = np.array(gpos).reshape((1))
        self.gvel = np.array(gvel).reshape((1))
    
    def ndof(self):
        return 1

    def pose(self):
        """
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.pose()
        array([[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
                  0.00000000e+00],
               [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]])
        """
        return Hg.rotz(self.gpos[0])

    def ipose(self):
        """
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.ipose()
        array([[  7.96326711e-04,   9.99999683e-01,   0.00000000e+00,
                  0.00000000e+00],
               [ -9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]])

        """
        return Hg.rotz(-self.gpos[0])

    def jacobian(self):
        """
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.jacobian()
        array([[ 0.],
               [ 0.],
               [ 1.],
               [ 0.],
               [ 0.],
               [ 0.]])
        """
        return np.array([[0.], [0.], [1.], [0.], [0.], [0.]])

    def djacobian(self):
        """
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.djacobian()
        array([[ 0.],
               [ 0.],
               [ 0.],
               [ 0.],
               [ 0.],
               [ 0.]])

        """
        return np.zeros((6,1))

