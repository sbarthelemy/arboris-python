# coding=utf-8
"""
Functions for working with homogeneous matrices.

toto
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")
import numpy as np


tol=1e-12

def transl(vec):
    """
    Homogeneous matrix of a translation.

    INPUT: ``vec`` - coordinates of the translation vector in 3d space

    OUTPUT: Homogeneous matrix of the translation
    """
    return np.vstack( (np.hstack((np.eye(3),np.array(vec).reshape(3,1))),
                       [0,0,0,1]))


def rotzyx(angles):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Ry * Rx

    example:
    >>> rotzyx((3.14/6, 3.14/6, 3.14/6))
    array([[ 0.75022984, -0.21653948,  0.6247126 ,  0.        ],
           [ 0.43287992,  0.8750575 , -0.21653948,  0.        ],
           [-0.4997701 ,  0.43287992,  0.75022984,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    """
    
    sx = np.sin(angles[0])
    cx = np.cos(angles[0])
    sy = np.sin(angles[1])
    cy = np.cos(angles[1])
    sz = np.sin(angles[2])
    cz = np.cos(angles[2])
    return np.array(
        [[ cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx, 0.],
         [ sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx, 0.],
         [-sy   , cy*sx         , cy*cx         , 0.],
         [ 0.   , 0.            , 0.            , 1.]])
         
         
def rotzy(angles):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Ry

    example:
    >>> rotzy((3.14/6, 3.14/6))

    """
    sy = np.sin(angles[0])
    cy = np.cos(angles[0])
    sz = np.sin(angles[1])
    cz = np.cos(angles[1])
    return np.array(
        [[ cz*cy,-sz, cz*sy, 0.],
         [ sz*cy, cz, sz*sy, 0.],
         [-sy   , 0., cy   , 0.],
         [ 0.   , 0., 0.   , 1.]])


def rotzx(angles):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Rx

    example:
    >>> rotzy((3.14/6, 3.14/6))

    """
    sx = np.sin(angles[0])
    cx = np.cos(angles[0])
    sz = np.sin(angles[1])
    cz = np.cos(angles[1])
    return np.array(
        [[ cz,-sz*cx, sz*sx, 0.],
         [ sz, cz*cx,-cz*sx, 0.],
         [ 0., sx   , cx   , 0.],
         [ 0., 0.   , 0.   , 1.]])
         
         
def rotyx(angles):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Ry * Rx

    example:
    >>> rotzy((3.14/6, 3.14/6))

    """
    sx = np.sin(angles[0])
    cx = np.cos(angles[0])
    sy = np.sin(angles[1])
    cy = np.cos(angles[1])
    return np.array(
        [[ cy, sy*sx, sy*cx, 0.],
         [ 0., cx   ,-sx   , 0.],
         [-sy, cy*sx, cy*cx, 0.],
         [ 0., 0.   , 0.   , 1.]])

def rotx(angle):
    """
    Homogeneous matrix of a rotation around the x-axis
   
    example:
    >>> rotx(3.14/6)
    array([[ 1.        ,  0.        ,  0.        ,  0.        ],
           [ 0.        ,  0.86615809, -0.4997701 ,  0.        ],
           [ 0.        ,  0.4997701 ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    ca = np.cos(angle)
    sa = np.sin(angle)
    H = np.array(
        [[1,  0,   0,  0],
         [0, ca, -sa,  0],
         [0, sa,  ca,  0],
         [0,  0,   0,  1]])
    return H

def roty(angle):
    """
    Homogeneous matrix of a rotation around the y-axis

    example:
    >>> roty(3.14/6)
    array([[ 0.86615809,  0.        ,  0.4997701 ,  0.        ],
           [ 0.        ,  1.        ,  0.        ,  0.        ],
           [-0.4997701 ,  0.        ,  0.86615809,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    ca = np.cos(angle)
    sa = np.sin(angle)
    H = np.array(
        [[ ca,  0,  sa,  0],
         [  0,  1,   0,  0],
         [-sa,  0,  ca,  0],
         [  0,  0,   0,  1]])
    return H

def rotz(angle):
    """
    Rotation around the z-axis
    example:
    >>> rotz(3.14/6)
    array([[ 0.86615809, -0.4997701 ,  0.        ,  0.        ],
           [ 0.4997701 ,  0.86615809,  0.        ,  0.        ],
           [ 0.        ,  0.        ,  1.        ,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])

    """
    ca = np.cos(angle)
    sa = np.sin(angle)
    H = np.array(
        [[ca, -sa, 0, 0],
         [sa,  ca, 0, 0],
         [ 0,   0, 1, 0],
         [ 0,   0, 0, 1]])
    return H

def ishomogeneousmatrix(H, tol=tol):
    """
    Return true if input is an homogeneous matrix
    """
    return (H.shape == (4,4)) \
        and (np.abs(np.linalg.det(H[0:3,0:3])-1)<=tol) \
        and (H[3,0:4]==[0,0,0,1]).all()

def checkishomogeneousmatrix(H, tol=tol):
    """
    Raise an error if input is not an homogeneous matrix
    """
    if not ishomogeneousmatrix(H, tol):
        raise ValueError("{H} is not an homogeneous matrix".format(H=H))

def inv(H):
    """
    Inverse an homogeneous matrix
    """
    R = H[0:3,0:3]
    p = H[0:3,3:4]
    return np.vstack( (np.hstack((R.T,-np.dot(R.T,p))), [0,0,0,1]))

def adjoint(H):
    """
    Return the adjoint of the homogeneous matrix
    """
    R = H[0:3,0:3]
    p = H[0:3,3:4]
    pxR = np.dot(
        np.array(
            [[    0, -p[2],  p[1]],
             [ p[2],     0, -p[0]],
             [-p[1],  p[0],     0]]),
        R)
    return np.vstack((
        np.hstack((R  , np.zeros((3,3)))),
        np.hstack((pxR, R))))

def iadjoint(H):
    """
    Return the adjoint of the inverse homogeneous matrix
    """
    return adjoint(inv(H))

if __name__ == "__main__":
    import doctest
    doctest.testmod()

