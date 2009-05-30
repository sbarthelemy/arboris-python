# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy import array, zeros, eye, dot, sin, cos, dot
from abc import ABCMeta, abstractmethod
import homogeneousmatrix
from misc import NamedObject
from arboris import Joint

class LinearConfigurationSpaceJoint(Joint):
    """
    joints whose space is diffeomorph to the real set.
    """

    def integrate(self, dt):
        self.gpos += dt * self.gvel



class FreeJoint(Joint):

    """Free joint (6-dof)
    """
    def __init__(self, gpos=None, gvel=None, frames=None, name=None):
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
        if gpos is None:
            gpos = eye(4)
        if gvel is None:
            gvel = zeros((6))
        self.gpos = array(gpos).reshape((4,4))
        self.gvel = array(gvel).reshape((6))
        Joint.__init__(self, frames, name)
    @property
    def ndof(self):
        return 6
    
    @property
    def pose(self):
        return self.gpos.copy()

    @property
    def twist(self):
        return self.gvel.copy()

    @property
    def jacobian(self):
        return eye(6)

    @property
    def djacobian(self):
        return zeros((6,6))

    def integrate(self, dt):
        from twistvector import exp
        self.gpos = dot(self.gpos, exp( dt*self.gvel))

#class PivotJoint(Joint):
#
#    """Pivot (2-dof)
#    """
#
#class BallJoint(Joint):
#
#    """Ball and socket (3-dof)
#    """

class RzRyRxJoint(LinearConfigurationSpaceJoint):
    """Ball and socket (3-dof) implemented with 3 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry*Rx
    """
    def __init__(self, gpos=[0.,0.,0.], gvel=[0.,0.,0.], frames=None, name=None):
        self.gpos = array(gpos).reshape((3))
        self.gvel = array(gvel).reshape((3))
        Joint.__init__(self,  frames, name)
    @property
    def ndof(self):
        return 3

    @property
    def pose(self):
        return homogeneousmatrix.rotzyx(self.gpos)

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        return array(
            [[ 1.   ,  0. , -sy    ],
             [ 0.   , cx  ,  sx*sy ],
             [ 0.   ,-sx  ,  cx*cy ],
             [ 0.   ,  0. ,  0.    ],
             [ 0.   ,  0. ,  0.    ],
             [ 0.   ,  0. ,  0.    ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        dx = self.gvel[0]
        dy = self.gvel[1]
        return array(
            [[ 0.   , 0.    ,-dy*cy             ],
             [ 0.   ,-dx*sx , dx*cx*sy+dy*sx*cy ],
             [ 0.   ,-dx*cx ,-dx*sx*cy-dy*cx*sy ],
             [ 0.   ,  0. ,  0.    ],
             [ 0.   ,  0. ,  0.    ],
             [ 0.   ,  0. ,  0.    ]])


class RzRyJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], frames=None, name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self,  frames, name)
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotzy(self.gpos)

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sy = sin(self.gpos[0])
        cy = cos(self.gpos[0])
        return array(
            [[ 0.   , -sy ],
             [ 1.   ,  0. ],
             [ 0.   ,  cy ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ]])
    
    @property
    def djacobian(self):
        sy = sin(self.gpos[0])
        cy = cos(self.gpos[0])
        dy = self.gvel[0]
        return array(
            [[ 0.   ,-dy*cy ],
             [ 0.   , 0.    ],
             [ 0.   ,-dy*sy ],
             [ 0.   ,  0.   ],
             [ 0.   ,  0.   ],
             [ 0.   ,  0.   ]])
             
             
class RzRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Rx
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], frames=None, name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self,  frames, name)
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotzx(self.gpos)

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        return array(
            [[ 1.   ,  0. ],
             [ 0.   ,  sx ],
             [ 0.   ,  cx ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        dx = self.gvel[0]
        return array(
            [[ 0.   , 0.    ],
             [ 0.   , dx*cx ],
             [ 0.   ,-dx*sx ],
             [ 0.   ,  0.   ],
             [ 0.   ,  0.   ],
             [ 0.   ,  0.   ]])
             
             
class RyRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], frames=None, name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self,  frames, name)
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotyx(self.gpos)

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        return array(
            [[ 1.   ,  0. ],
             [ 0.   ,  cx ],
             [ 0.   , -sx ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ],
             [ 0.   ,  0. ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[0])
        cx = cos(self.gpos[0])
        dx = self.gvel[0]
        return array(
            [[ 0.   , 0.    ],
             [ 0.   ,-dx*sx ],
             [ 0.   ,-dx*cx ],
             [ 0.   , 0.    ],
             [ 0.   , 0.    ],
             [ 0.   , 0.    ]])
             

class RzJoint(LinearConfigurationSpaceJoint):

    """Hinge (1-dof) with axis in the z-direction
    """
    def __init__(self, gpos=0., gvel=0., frames=None, name=None):
        """
        example:
        >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.gpos
        array([ 1.57])
        >>> j.gvel
        array([ 1.])
        """
        self.gpos = array(gpos).reshape((1))
        self.gvel = array(gvel).reshape((1))
        Joint.__init__(self,  frames, name)
    @property
    def ndof(self):
        return 1

    @property
    def pose(self):
        """
        >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.pose
        array([[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
                  0.00000000e+00],
               [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]])
        """
        return homogeneousmatrix.rotz(self.gpos[0])

    @property
    def ipose(self):
        """
        >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.ipose
        array([[  7.96326711e-04,   9.99999683e-01,   0.00000000e+00,
                  0.00000000e+00],
               [ -9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]])

        """
        return homogeneousmatrix.rotz(-self.gpos[0])

    @property
    def jacobian(self):
        """
        >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.jacobian
        array([[ 0.],
               [ 0.],
               [ 1.],
               [ 0.],
               [ 0.],
               [ 0.]])
        """
        return array([[0.], [0.], [1.], [0.], [0.], [0.]])

    @property
    def djacobian(self):
        """
        >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.djacobian
        array([[ 0.],
               [ 0.],
               [ 0.],
               [ 0.],
               [ 0.],
               [ 0.]])

        """
        return zeros((6,1))

if __name__ == "__main__":
    import doctest
    doctest.testmod()

