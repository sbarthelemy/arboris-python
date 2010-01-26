# coding=utf-8
"""Different kinds of joints"""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from numpy import array, zeros, eye, sin, cos, dot
import homogeneousmatrix
from core import Joint, LinearConfigurationSpaceJoint

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
        if gpos is None:
            gpos = eye(4)
        if gvel is None:
            gvel = zeros((6))
        self.gpos = array(gpos).reshape((4,4))
        self.gvel = array(gvel).reshape((6))
        Joint.__init__(self, name)
        
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

    def integrate(self, gvel, dt):
        from twistvector import exp
        self.gvel = gvel
        self.gpos = dot(self.gpos, exp( dt*self.gvel))

class RzRyRxJoint(LinearConfigurationSpaceJoint):
    """Ball and socket (3-dof) joint implemented with 3 serial hinges

    the resulting homogeneous matrix is given by `H_{01} = Rz Ry Rx`
    """
    @property
    def ndof(self):
        return 3

    @property
    def pose(self):
        return homogeneousmatrix.rotzyx(
            self.gpos[0],self.gpos[1],self.gpos[2])

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sx = sin(self.gpos[2])
        cx = cos(self.gpos[2])
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        return array(
            [[ -sy    ,   0. , 1. ],
             [  sx*cy ,  cx  , 0. ],
             [  cx*cy , -sx  , 0. ],
             [  0.    ,   0. , 0. ],
             [  0.    ,   0. , 0. ],
             [  0.    ,   0. , 0. ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[2])
        cx = cos(self.gpos[2])
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        dx = self.gvel[2]
        dy = self.gvel[1]
        return array(
            [[-dy*cy             , 0.    ,  0. ],
             [ dx*cx*cy-dy*sx*sy ,-dx*sx ,  0. ],
             [-dx*sx*cy-dy*cx*sy ,-dx*cx ,  0. ],
             [ 0.                ,  0.   ,  0. ],
             [ 0.                ,  0.   ,  0. ],
             [ 0.                ,  0.   ,  0. ]])


class RzRyJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotzy(self.gpos[0], self.gpos[1])

    @property
    def jacobian(self):
        """
        T_n/r = 
        """   
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        return array(
            [[ -sy , 0. ],
             [  0. , 1. ],
             [  cy , 0. ],
             [  0. , 0. ],
             [  0. , 0. ],
             [  0. , 0. ]])
    
    @property
    def djacobian(self):
        sy = sin(self.gpos[1])
        cy = cos(self.gpos[1])
        dy = self.gvel[1]
        return array(
            [[ -dy*cy , 0.  ],
             [  0.    , 0.  ],
             [ -dy*sy , 0.  ],
             [   0.   , 0.  ],
             [   0.   , 0.  ],
             [   0.   , 0.  ]])
             
             
class RzRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Rx
    """
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotzx(self.gpos[0], self.gpos[1])

    @property
    def jacobian(self):
        sx = sin(self.gpos[1])
        cx = cos(self.gpos[1])
        return array(
            [[ 0. , 1. ],
             [ sx , 0. ],
             [ cx , 0. ],
             [ 0. , 0. ],
             [ 0. , 0. ],
             [ 0. , 0. ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[1])
        cx = cos(self.gpos[1])
        dx = self.gvel[1]
        return array(
            [[  0.    , 0.  ],
             [  dx*cx , 0.  ],
             [ -dx*sx , 0.  ],
             [   0.   , 0.  ],
             [   0.   , 0.  ],
             [   0.   , 0.  ]])
             
             
class RyRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    @property
    def ndof(self):
        return 2

    @property
    def pose(self):
        return homogeneousmatrix.rotyx(self.gpos[0], self.gpos[1])

    @property
    def jacobian(self):
        sx = sin(self.gpos[1])
        cx = cos(self.gpos[1])
        return array(
            [[  0. , 1. ],
             [  cx , 0. ],
             [ -sx , 0. ],
             [  0. , 0. ],
             [  0. , 0. ],
             [  0. , 0. ]])
    
    @property
    def djacobian(self):
        sx = sin(self.gpos[1])
        cx = cos(self.gpos[1])
        dx = self.gvel[1]
        return array(
            [[ 0.    , 0. ],
             [-dx*sx , 0. ],
             [-dx*cx , 0. ],
             [ 0.    , 0. ],
             [ 0.    , 0. ],
             [ 0.    , 0. ]])
             

class RzJoint(LinearConfigurationSpaceJoint):
    """Hinge (1-dof) with axis in the z-direction
    
    example:
    
    >>> j = RzJoint(gpos = 3.14/2., gvel = 1.)
    >>> j.gpos
    array([ 1.57])
    >>> j.gvel
    array([ 1.])
   
    """
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

class RyJoint(LinearConfigurationSpaceJoint):
    """Hinge (1-dof) with axis in the y-direction.
    """
    @property
    def ndof(self):
        return 1

    @property
    def pose(self):
        return homogeneousmatrix.roty(self.gpos[0])

    @property
    def ipose(self):
        return homogeneousmatrix.roty(-self.gpos[0])

    @property
    def jacobian(self):
        return array([[0.], [1.], [0.], [0.], [0.], [0.]])

    @property
    def djacobian(self):
        return zeros((6,1))
        
class RxJoint(LinearConfigurationSpaceJoint):
    """Hinge (1-dof) with axis in the x-direction
    """
    @property
    def ndof(self):
        return 1

    @property
    def pose(self):
        return homogeneousmatrix.rotx(self.gpos[0])

    @property
    def ipose(self):
        return homogeneousmatrix.rotx(-self.gpos[0])

    @property
    def jacobian(self):
        return array([[1.], [0.], [0.], [0.], [0.], [0.]])

    @property
    def djacobian(self):
        return zeros((6,1))


class TxTyTzJoint(LinearConfigurationSpaceJoint):
    """Triple prismatic joint (3-dof).

    the resulting homogeneous matrix is given by H = Tx*Ty*Tz.
    """
    @property
    def ndof(self):
        return 3

    @property
    def pose(self):
        return homogeneousmatrix.transl(
                self.gpos[0], self.gpos[1], self.gpos[2])

    @property
    def jacobian(self):
        return array(
            [[  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  1. , 0. , 0. ],
             [  0. , 1. , 0. ],
             [  0. , 0. , 1. ]])

    @property
    def djacobian(self):
        return array(
            [[  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  0. , 0. , 0. ],
             [  0. , 0. , 0. ]])
