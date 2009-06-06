# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy import array, zeros, eye, dot, sin, cos          #no more need of sin, cos
from abc import ABCMeta, abstractmethod
import homogeneousmatrix
cimport homogeneousmatrix
from misc import NamedObject
from arboris import Joint

#mes modifs....
cimport numpy as np
DTYPE = np.float 
ctypedef np.float_t DTYPE_t
cdef extern from "math.h":
    float cosf(float theta)
    float sinf(float theta)


#must see Joint class to enhance this one...
class LinearConfigurationSpaceJoint(Joint):
    """
    joints whose space is diffeomorph to the real set.
    """

    def integrate(self, dt):
        self.gpos += dt * self.gvel



cdef class FreeJoint(Joint):

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

    property ndof:
        def __get__(self):   # This is called when the property is read.
            return 6
    
    property pose:
        def __get__(self):
            return self.gpos.copy()

    property twist:
        def __get__(self):
            return self.gvel.copy()

    property jacobian:
        def __get__(self):
            return eye(6)

    property djacobian:
        def __get__(self):
            return zeros((6,6))

    def integrate(self, dt):         #must see twistvector, exp dot to enhance this methode
        from twistvector import exp  #twistvector must be declared 'cdef exter class tewistsvector'
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



cdef class RzRyRxJoint(LinearConfigurationSpaceJoint):
    """Ball and socket (3-dof) implemented with 3 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry*Rx
    """
    def __init__(self, gpos=[0.,0.,0.], gvel=[0.,0.,0.], name=None):
        self.gpos = array(gpos).reshape((3))
        self.gvel = array(gvel).reshape((3))
        Joint.__init__(self, name)

    property ndof:  #mes modifs.......
        def __get__(self):
            return 3

    property pose:
        def __get__(self):
            return homogeneousmatrix.rotzyx(self.gpos)
    
    cdef _getjacobian(self):
    
        """
        T_n/r = 
        """   
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])
        cdef float sy = sinf(self.gpos[1])
        cdef float cy = cosf(self.gpos[1])

#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 1.   ,  0. , -sy    ],
#             [ 0.   , cx  ,  sx*sy ],
#             [ 0.   ,-sx  ,  cx*cy ],
#             [ 0.   ,  0. ,  0.    ],
#             [ 0.   ,  0. ,  0.    ],
#             [ 0.   ,  0. ,  0.    ]),   dtype=DTYPE)

        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,3),dtype=DTYPE))
        H[0,0]=1
        H[0,2]=-sy
        H[1,1]=cx
        H[1,2]=sx*sy
        H[2,1]=-sx
        H[2,2]=cx*cy
        return value

    #we used this method to preserve the use of proprety:(
    #eventally look further if we could write cpdef jacobian.... and at the same time preserve proprety
    #pay attention about names...
    property jacobian:
        def __get__(self):
            return self._getjacobian()         
    #same thing for djacobian
    cdef _getdjacobian(self):
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])
        cdef float sy = sinf(self.gpos[1])
        cdef float cy = cosf(self.gpos[1])
        cdef float dx = self.gvel[0]
        cdef float dy = self.gvel[1]
#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 0.   , 0.    ,-dy*cy             ],
#             [ 0.   ,-dx*sx , dx*cx*sy+dy*sx*cy ],
#             [ 0.   ,-dx*cx ,-dx*sx*cy-dy*cx*sy ],
#             [ 0.   ,  0. ,  0.    ],
#             [ 0.   ,  0. ,  0.    ],
#             [ 0.   ,  0. ,  0.    ]), dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,3),dtype=DTYPE))
        H[0,2]= -dy*cy
        H[1,1]= -dx*sx
        H[1,2]=  dx*cx*sy+dy*sx*cy
        H[2,1]= -dx*cy
        H[2,2]= -dx*sx*cy-dy*cx*sy

        return value

    property djacobian:
        def __get__(self):
            return self._getdjacobian()         

cdef class RzRyJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):          #maybe should think about using __cinit__()
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    property ndof:
        def __get__(self):
            return 2

    property pose :
        def __get__(self):
            return homogeneousmatrix.rotzy(self.gpos)

    cdef _getjacobian(self):
        """
        T_n/r = 
        """   
        cdef float sy = sinf(self.gpos[0])
        cdef float cy = cosf(self.gpos[0])
#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 0.   , -sy ],
#             [ 1.   ,  0. ],
#             [ 0.   ,  cy ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ]),  dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[0,1]=-sy
        H[1,0]=1.
        H[2,1]=cy
        return value

    property jacobian:
        def __get__(self):
            return self._getjacobian()

    cdef _getdjacobian(self):
        cdef float sy = sinf(self.gpos[0])
        cdef float cy = cosf(self.gpos[0])
        cdef float dy = self.gvel[0]
#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 0.   ,-dy*cy ],
#             [ 0.   , 0.    ],
#             [ 0.   ,-dy*sy ],
#             [ 0.   ,  0.   ],
#             [ 0.   ,  0.   ],
#             [ 0.   ,  0.   ]), dtype=DTYPE)

        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[0,1]=-dy*cy
        H[2,1]=-dy*sy
        return value
    property djacobian:
        def __get__(self):
            return self._getdjacobian()

cdef class RzRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Rx
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):      #todo:maybe _cinit() with precaution...
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    property ndof:
        def __get__(self):
            return 2

    property pose:
        def __get__(self):
            return homogeneousmatrix.rotzx(self.gpos)

    cdef _getjacobian(self):
        """
        T_n/r = 
        """   
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])

#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 1.   ,  0. ],
#             [ 0.   ,  sx ],
#             [ 0.   ,  cx ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ]), dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[0,0]=1.
        H[1,1]=sx
        H[2,1]=cx

        return value

    property jacobian:
        def __get__(self):
            return self._getjacobian


    cdef _getdjacobian(self):
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])
        cdef float dx = self.gvel[0]
#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 0.   , 0.    ],
#             [ 0.   , dx*cx ],
#             [ 0.   ,-dx*sx ],
#             [ 0.   ,  0.   ],
#             [ 0.   ,  0.   ],
#             [ 0.   ,  0.   ]), dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[1,1]=dx*cx
        H[2,1]=-dx*sx

        return value
    property djacobian:
        def __get__(self):
            return self._getdjacobian
             
cdef class RyRxJoint(LinearConfigurationSpaceJoint):
    """Fingered Ball (2-dof) implemented with 2 serial hinges

    the resulting homogeneous matrix is given by H = Rz*Ry
    """
    def __init__(self, gpos=[0.,0.], gvel=[0.,0.], name=None):
        self.gpos = array(gpos).reshape((2))
        self.gvel = array(gvel).reshape((2))
        Joint.__init__(self, name)

    property ndof:
        def __get__(self):
            return 2

    property pose:
        def __get__(self):
            return homogeneousmatrix.rotyx(self.gpos)

    cdef _getjacobian(self):
        """
        T_n/r = 
        """   
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])

#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 1.   ,  0. ],
#             [ 0.   ,  cx ],
#             [ 0.   , -sx ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ],
#             [ 0.   ,  0. ]), dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[0,0]=1.
        H[1,1]=cx
        H[2,1]=-sx
        return value

    property jacobian:
        def __get__(self):
            return self._getjacobian 

    cdef _getdjacobian(self):
        cdef float sx = sinf(self.gpos[0])
        cdef float cx = cosf(self.gpos[0])
        cdef float dx = self.gvel[0]
#        cdef np.ndarray[DTYPE_t, ndim=2] value = np.array (
#            ([ 0.   , 0.    ],
#             [ 0.   ,-dx*sx ],
#             [ 0.   ,-dx*cx ],
#             [ 0.   , 0.    ],
#             [ 0.   , 0.    ],
#             [ 0.   , 0.    ]), dtype=DTYPE)
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,2),dtype=DTYPE))
        H[1,1]=-dx*sx
        H[2,1]=-dx*cx
        return value
    property djacobian:
        def __get__(self):
            return self._getdjacobian()

cdef class RzJoint(LinearConfigurationSpaceJoint):

    """Hinge (1-dof) with axis in the z-direction
    """
    def __init__(self, gpos=0., gvel=0., name=None):
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
        Joint.__init__(self, name)
    
    property ndof:
        def __get__(self):
            return 1

    property pose:
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
        def pose(self):
            return homogeneousmatrix.rotz(self.gpos[0])

    property ipose:
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
        def __get__(self):
            return homogeneousmatrix.rotz(-self.gpos[0])


    cdef _getjacobian(self):
#        cdef np.ndarray[DTYPE_t, ndim=1] value = np.array (([0.], [0.], [1.], [0.], [0.], [0.]))
        cdef np.ndarray[DTYPE_t, ndim=2] value = zeros ((6,1),dtype=DTYPE))
        value[2,0]=1.
        return value

    property jacobian:
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
        def __get__(self):
            return self._getjacobian()


    cdef _getdjacobian(self):
        cdef np.ndarray[DTYPE_t, ndim=1] value = zeros((6,1))
        return value
    property djacobian:
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
        def __get__(self):
            return self._getdjacobian()

if __name__ == "__main__":
    import doctest
    doctest.testmod()

