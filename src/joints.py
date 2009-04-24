# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy import array, zeros, eye, dot, sin, cos
from rigidmotion import RigidMotion
from abc import ABCMeta, abstractmethod
import homogeneousmatrix

class Joint(RigidMotion):
    """any joint
    H
    T_
    """    
    __metaclass__ = ABCMeta

    def __init__(self, name=None):
        self._name = name

    def twist(self):
        return dot(self.jacobian(), self.gvel)


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

    @abstractmethod
    def integrate(self, dt):
        pass

class RealJoint(Joint):
    """
    joints whose space is diffeomorph to the real set.
    """

    def integrate(self, dt):
        self.gpos += dt * self.gvel



class FreeJoint(Joint):

    """Free joint (6-dof)
    """
    def __init__(self, gpos=None, gvel=None, name=None):
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
            gpos = eye(4)
        if gvel == None:
            gvel = zeros((6))
        self.gpos = array(gpos).reshape((4,4))
        self.gvel = array(gvel).reshape((6))
        Joint.__init__(self, name)

    def ndof(self):
        return 6
    
    def pose(self):
        return self.gpos.copy()

    def twist(self):
        return self.gvel.copy()

    def jacobian(self):
        return eye(6)

    def djacobian(self):
        return zeros((6,6))

    def integrate(self, dt):
        from vectortwist import exp
        self.gpos = self.gpos * exp( dt*self.gvel)

#class PivotJoint(Joint):
#
#    """Pivot (2-dof)
#    """
#
#class BallJoint(Joint):
#
#    """Ball and socket (3-dof)
#    """

class RzRyRxJoint(RealJoint):
    """Ball and socket (3-dof) implemented with 3 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry*Rx
    """
    def __init__(self, gpos=[0.,0.,0.], gvel=[0.,0.,0.], name=None):
        self.gpos = array(gpos).reshape((3))
        self.gvel = array(gvel).reshape((3))
        Joint.__init__(self, name)

    def ndof(self):
        return 3

    def pose(self):
        return homogeneousmatrix.rotzyx(self.gpos)

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


class RzRyJoint(RealJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    def ndof(self):
        return 2

    def pose(self):
        return homogeneousmatrix.rotzy(self.gpos)

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
             
             
class RzRxJoint(RealJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Rx
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    def ndof(self):
        return 2

    def pose(self):
        return homogeneousmatrix.rotzx(self.gpos)

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
             
             
class RyRxJoint(RealJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    def ndof(self):
        return 2

    def pose(self):
        return homogeneousmatrix.rotyx(self.gpos)

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
             
             
class HingeJoint(RealJoint):

    """Hinge (1-dof) with axis in the z-direction
    """
    def __init__(self, gpos=0., gvel=0., name=None):
        """
        example:
        >>> j = HingeJoint(gpos = 3.14/2., gvel = 1.)
        >>> j.gpos
        array([ 1.57])
        >>> j.gvel
        array([ 1.])
        """
        self.gpos = array(gpos).reshape((1))
        self.gvel = array(gvel).reshape((1))
        Joint.__init__(self, name)
    
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
        return homogeneousmatrix.rotz(self.gpos[0])

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
        return homogeneousmatrix.rotz(-self.gpos[0])

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
        return array([[0.], [0.], [1.], [0.], [0.], [0.]])

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
        return zeros((6,1))

if __name__ == "__main__":
    import doctest
    doctest.testmod()

