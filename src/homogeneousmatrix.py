# coding=utf-8
import numpy as np
"""
Functions for working with homogeneous matrices.

toto
"""
tol=1e-12

def transl(vec):
    """
    Homogeneous matrix of a translation.

    INPUT: ``vec`` - coordinates of the translation vector in 3d space

    OUTPUT: Homogeneous matrix of the translation
    """
    return np.vstack( (np.hstack((np.eye(3),np.array(vec).reshape(3,1))), [0,0,0,1]))


def rotx(theta):
    """
    Homogeneous matrix of a rotation around the x-axis
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    H = np.array(
        [[1,  0,   0,  0],
         [0, ct, -st,  0],
         [0, st,  ct,  0],
         [0,  0,   0,  1]])
    return H

def rotz(theta):
    """
    Rotation around the x-axis
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    H = np.array(
        [[ct, -st, 0, 0],
         [st,  ct, 0, 0],
         [ 0,   0, 1, 0],
         [ 0,   0, 0, 1]])
    return H

def ishomogeneousmatrix(H, tol=tol):
    """
    Return true if input is an homogeneous matrix
    """
    return (H.shape == (4,4)) and (np.abs(np.linalg.det(H[0:3,0:3])-1)<=tol) and (H[3,0:4]==[0,0,0,1]).all()

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
