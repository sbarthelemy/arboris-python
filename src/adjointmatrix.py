import numpy as np
"""
H = [ R r
      0 1 ]    
Ad(H) = [  R   0
          rxR  R ]
"""

def isadjointmatrix(a):
    """
    Return true if a is an adjoint matrix
    """
#todo: do a better check
    return (a.shape == (6,6)) and (
        np.linalg.det(a[0:3,0:3])==1) and (
        a[0:3,0:3]==a[3:6,3:6]).all() and (
        a[0:3,3:6]==np.zeros((3,3))).all() 


def inv(Ad):
    """
    Invert an adjoint matrix
    """
    R = Ad[0:3,0:3].transpose()
    pxR = Ad[3:6,0:3].transpose()
    return np.vstack((
        np.hstack((R  , np.zeros((3,3)))),
        np.hstack((pxR, R))))


