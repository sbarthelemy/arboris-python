# coding=utf-8
"""
Functions for working with homogeneous matrices.

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

import numpy
import numpy as np
from numpy import array, zeros, empty, dot
cimport numpy

DTYPE = numpy.float
#ctypedef numpy.float_t DTYPE_t

cdef extern from "math.h":
    double cos(double angle)
    double sin(double angle)

cdef DTYPE_t tol=1e-12

cpdef transl(DTYPE_t t_x, DTYPE_t t_y, DTYPE_t t_z):
    """
    Homogeneous matrix of a translation.

    INPUT: ``vec`` - coordinates of the translation vector in 3d space

    OUTPUT: Homogeneous matrix of the translation

    Example:

    >>> transl(1., 2., 3.)
    array([[ 1.,  0.,  0.,  1.],
           [ 0.,  1.,  0.,  2.],
           [ 0.,  0.,  1.,  3.],
           [ 0.,  0.,  0.,  1.]])

    """
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = empty((4,4), dtype=DTYPE)
    H[0,0] = 1.
    H[0,1] = 0.
    H[0,2] = 0.
    H[0,3] = t_x
    H[1,0] = 0.
    H[1,1] = 1
    H[1,2] = 0.
    H[1,3] = t_y
    H[2,0] = 0.
    H[2,1] = 0.
    H[2,2] = 1
    H[2,3] = t_z
    H[3,0] = 0.
    H[3,1] = 0.
    H[3,2] = 0.
    H[3,3] = 1
    return H


cpdef rotzyx(DTYPE_t angle_z, DTYPE_t angle_y, DTYPE_t angle_x) :
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Ry * Rx

    example:
    >>> rotzyx(3.14/6, 3.14/4, 3.14/3)
    array([[ 0.61271008,  0.27992274,  0.73907349,  0.        ],
           [ 0.35353151,  0.73930695, -0.57309746,  0.        ],
           [-0.70682518,  0.61242835,  0.35401931,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    cdef DTYPE_t sz = sin(angle_z)
    cdef DTYPE_t cz = cos(angle_z)
    cdef DTYPE_t sy = sin(angle_y)
    cdef DTYPE_t cy = cos(angle_y)
    cdef DTYPE_t sx = sin(angle_x)
    cdef DTYPE_t cx = cos(angle_x)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = cz*cy
    H[0,1] = cz*sy*sx-sz*cx
    H[0,2] = cz*sy*cx+sz*sx
    H[1,0] = sz*cy
    H[1,1] = sz*sy*sx+cz*cx
    H[1,2] = sz*sy*cx-cz*sx
    H[2,0] = -sy
    H[2,1] = cy*sx
    H[2,2] = cy*cx
    H[3,3] = 1
    return H


cpdef rotzy(DTYPE_t angle_z, DTYPE_t angle_y):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Ry

    example:
    >>> rotzy(3.14/6, 3.14/4)
    array([[ 0.61271008, -0.4997701 ,  0.61222235,  0.        ],
           [ 0.35353151,  0.86615809,  0.35325009,  0.        ],
           [-0.70682518,  0.        ,  0.70738827,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    cdef DTYPE_t sz = sin(angle_z)
    cdef DTYPE_t cz = cos(angle_z)
    cdef DTYPE_t sy = sin(angle_y)
    cdef DTYPE_t cy = cos(angle_y)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = cz*cy
    H[0,1] = -sz
    H[0,2] = cz*sy
    H[1,0] = sz*cy
    H[1,1] = cz
    H[1,2] = sz*sy
    H[2,0] = -sy
    H[2,2] = cy
    H[3,3] = 1
    return H 


cpdef rotzx(DTYPE_t angle_z, DTYPE_t angle_x):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Rx

    example:
    >>> rotzx(3.14/6, 3.14/3)
    array([[ 0.86615809, -0.25011479,  0.43268088,  0.        ],
           [ 0.4997701 ,  0.43347721, -0.74988489,  0.        ],
           [ 0.        ,  0.86575984,  0.50045969,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef DTYPE_t sz = sin(angle_z)
    cdef DTYPE_t cz = cos(angle_z)
    cdef DTYPE_t sx = sin(angle_x)
    cdef DTYPE_t cx = cos(angle_x)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = cz
    H[0,1] = -sz*cx
    H[0,2] = sz*sx
    H[1,0] = sz
    H[1,1] = cz*cx
    H[1,2] = -cz*sx
    H[2,1] = sx
    H[2,2] = cx
    H[3,3] = 1
    return H


cpdef rotyx(DTYPE_t angle_y, DTYPE_t angle_x):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Ry * Rx

    example:
    >>> rotyx(3.14/4, 3.14/3)
    array([[ 0.70738827,  0.61194086,  0.35373751,  0.        ],
           [ 0.        ,  0.50045969, -0.86575984,  0.        ],
           [-0.70682518,  0.61242835,  0.35401931,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef DTYPE_t sy = sin(angle_y)
    cdef DTYPE_t cy = cos(angle_y)
    cdef DTYPE_t sx = sin(angle_x)
    cdef DTYPE_t cx = cos(angle_x)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = cy
    H[0,1] = sy*sx
    H[0,2] = sy*cx
    H[1,1] = cx
    H[1,2] = -sx
    H[2,0] = -sy
    H[2,1] = cy*sx
    H[2,2] = cy*cx
    H[3,3] = 1
    return H


cpdef rotx(DTYPE_t angle):
    """
    Homogeneous matrix of a rotation around the x-axis
   
    example:
    >>> rotx(3.14/6)
    array([[ 1.        ,  0.        ,  0.        ,  0.        ],
           [ 0.        ,  0.86615809, -0.4997701 ,  0.        ],
           [ 0.        ,  0.4997701 ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef DTYPE_t cx = cos(angle)
    cdef DTYPE_t sx = sin(angle)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = empty((4,4), dtype=DTYPE)
    H[0,0] = 1.
    H[0,1] = 0.
    H[0,2] = 0.
    H[0,3] = 0.
    H[1,0] = 0.
    H[1,1] = cx
    H[1,2] = -sx
    H[1,3] = 0.
    H[2,0] = 0.
    H[2,1] = sx
    H[2,2] = cx
    H[2,3] = 0.
    H[3,0] = 0.
    H[3,1] = 0.
    H[3,2] = 0.
    H[3,3] = 1.
    return H


cpdef roty(DTYPE_t angle):
    """
    Homogeneous matrix of a rotation around the y-axis

    example:
    >>> roty(3.14/6)
    array([[ 0.86615809,  0.        ,  0.4997701 ,  0.        ],
           [ 0.        ,  1.        ,  0.        ,  0.        ],
           [-0.4997701 ,  0.        ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef DTYPE_t ca = cos(angle)
    cdef DTYPE_t sa = sin(angle)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = ca
    H[0,2] = sa
    H[1,1] = 1.
    H[2,0] = -sa
    H[2,2] = ca
    H[3,3] = 1.
    return H

cpdef rotz(DTYPE_t angle):
    """
    Rotation around the z-axis
    example:
    >>> rotz(3.14/6)
    array([[ 0.86615809, -0.4997701 ,  0.        ,  0.        ],
           [ 0.4997701 ,  0.86615809,  0.        ,  0.        ],
           [ 0.        ,  0.        ,  1.        ,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef DTYPE_t ca = cos(angle)
    cdef DTYPE_t sa = sin(angle)
    cdef numpy.ndarray[DTYPE_t, ndim=2] H = zeros((4,4), dtype=DTYPE)
    H[0,0] = ca
    H[0,1] = -sa
    H[1,0] = sa
    H[1,1] = ca
    H[2,2] = 1.
    H[3,3] = 1.

    return H
#must ask about cythoning this one :'cause booleans are not the same in python and c
def ishomogeneousmatrix(H, tol=tol):
    """
    Return true if input is an homogeneous matrix
    """
    return (H.shape == (4,4)) \
        and (numpy.abs(numpy.linalg.det(H[0:3,0:3])-1)<=tol) \
        and (H[3,0:4]==[0,0,0,1]).all()

def checkishomogeneousmatrix(H, tol=tol):
    """
    Raise an error if input is not an homogeneous matrix
    """
    if not ishomogeneousmatrix(H, tol):
        raise ValueError("{H} is not an homogeneous matrix".format(H=H))

cpdef inv(numpy.ndarray H):
    """
    Inverse an homogeneous matrix
    >>> H = array(
    ...     [[ 0.70738827,  0.        , -0.70682518,  3.        ],
    ...      [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
    ...      [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
    ...      [ 0.        ,  0.        ,  0.        ,  1.        ]])
    >>> inv(H)
    array([[ 0.70738827,  0.61194086,  0.35373751, -6.3386158 ],
           [ 0.        ,  0.50045969, -0.86575984,  2.32696044],
           [-0.70682518,  0.61242835,  0.35401931, -2.09933441],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    cdef int i,j
    cdef numpy.ndarray[DTYPE_t, ndim=2] iH = empty((4,4), dtype=DTYPE)
    
    for i from 0 <= i < 3:
        for j from 0 <= j < 3:
            iH[i,j] = H[j,i]
    iH[0:3, 3] = -dot(iH[0:3, 0:3], H[0:3, 3])
    iH[3,0:3] = 0.
    iH[3,3] = 1.
    return iH


cpdef asym(DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z):
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

cpdef adjoint(numpy.ndarray H):
    """
    Return the adjoint of the homogeneous matrix

    >>> H = array(
    ...     [[ 0.70738827,  0.        , -0.70682518,  3.        ],
    ...      [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
    ...      [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
    ...      [ 0.        ,  0.        ,  0.        ,  1.        ]])
    >>> adjoint(H)
    array([[ 0.70738827,  0.        , -0.70682518,  0.        ,  0.        ,
             0.        ],
           [ 0.61194086,  0.50045969,  0.61242835,  0.        ,  0.        ,
             0.        ],
           [ 0.35373751, -0.86575984,  0.35401931,  0.        ,  0.        ,
             0.        ],
           [-1.64475426, -5.96533781, -1.64606451,  0.70738827,  0.        ,
            -0.70682518],
           [ 2.47572882,  2.59727952, -4.59618383,  0.61194086,  0.50045969,
             0.61242835],
           [-0.9937305 ,  1.50137907,  4.66458577,  0.35373751, -0.86575984,
             0.35401931]])

    """
    cdef int i,j
    cdef numpy.ndarray[DTYPE_t, ndim=2] Ad = zeros((6,6), dtype=DTYPE)
    Ad[0:3,0:3] = H[0:3,0:3]
    Ad[0:3,3:6] = 0.
    Ad[3:6,0:3] = dot(asym(H[0,3],H[1,3],H[2,3]), H[0:3,0:3])
    Ad[3:6,3:6] = H[0:3,0:3]
    return Ad


cpdef iadjoint(numpy.ndarray H):
    """
    Return the adjoint of the inverse homogeneous matrix
    """
    return adjoint(inv(H))

