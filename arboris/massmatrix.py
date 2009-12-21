# coding=utf-8
"""Functions for working with mass matrices.
"""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

import homogeneousmatrix as Hg
from numpy import diag, eye, dot, array
from numpy.linalg import eig, eigvals, det, norm

tol=1e-9

def ismassmatrix(M, tol=tol, semi=False):
    """Check whether M is a valid mass matrix.

    Return ``True`` if M is correctly shaped and symmetric positive
    definite.

    When ``semi`` is set to ``True``, positive *semi*-definite matrices
    are also considered valid.

    """
    common = M.shape == (6,6) and norm(M-M.T) <= tol and \
            (M[3:6,3:6] == M[3,3]*eye(3)).all()
    if not common:
        print (M[3:6,3:6] == M[3,3]*eye(3))
        print norm(M-M.T)
    if semi:
        return common and (eigvals(M)>=0.).all()
    else:
        return common and (eigvals(M)>0.).all()

def transport(M, H):
    """Transport (express) the mass matrix into another frame.
        
    :param M: the mass matrix expressed in the original frame (say, `a`)
    :type M: (6,6)-shaped array
    :param H: homogeneous matrix from the new frame (say `b`) to the
              original one: `H_{ab}`
    :type H: (4,4)-shaped array
    :rtype: (6,6)-shaped array
    
    **Example:**
    
    >>> M_a = diag((3.,2.,4.,1.,1.,1.))
    >>> H_ab = Hg.transl(1., 3., 0.)
    >>> M_b = transport(M_a, H_ab)
    >>> M_b
    array([[ 12.,  -3.,   0.,   0.,   0.,  -3.],
           [ -3.,   3.,   0.,   0.,   0.,   1.],
           [  0.,   0.,  14.,   3.,  -1.,   0.],
           [  0.,   0.,   3.,   1.,   0.,   0.],
           [  0.,   0.,  -1.,   0.,   1.,   0.],
           [ -3.,   1.,   0.,   0.,   0.,   1.]])
    
    """
    assert ismassmatrix(M)
    assert Hg.ishomogeneousmatrix(H)
    Ad = Hg.adjoint(H)
    return dot(Ad.T, dot(M, Ad))


def principalframe(M):
    """Find the principal frame of inertia of a mass matrix.

    :param M: mass matrix expressed in any frame (say `a`)
    :type M: (6,6)-shaped array
    :rtype: (4,4)-shaped array

    Returns the homogeneous matrix `H_{am}` to the principal inertia
    frame `m`

    **Example:**
    
    >>> M_a = diag((3.,2.,4.,1.,1.,1.))
    >>> H_ab = Hg.transl(1., 3., 0.)
    >>> M_b = transport(M_a, H_ab)
    >>> H_ba = principalframe(M_b)
    >>> dot(H_ab, H_ba)
    array([[ 1.,  0.,  0.,  0.],
           [ 0.,  1.,  0.,  0.],
           [ 0.,  0.,  1.,  0.],
           [ 0.,  0.,  0.,  1.]])
    
    """
    assert ismassmatrix(M)
    m = M[5,5]
    rx = M[0:3,3:6]/m
    H = eye(4)
    H[0:3,3] = array([rx[2,1],rx[0,2],rx[1,0]]) #TODO: use a function
    RSR = M[0:3,0:3] + m*dot(rx, rx)
    [S, R] = eig(RSR)
    if det(R)<0.:
        iI = array([[0,0,1],[0,1,0],[1,0,0]])
        R = dot(R, iI)
        S = dot(iI, dot(S, iI))
    H[0:3,0:3] = R
    return H

def box(half_extents, mass):
    """Mass matrix of an homogeneous parallelepiped.
    
    **Example:**
    
    >>> box((2.,4.,6.), 3.)
    array([[ 52.,   0.,   0.,   0.,   0.,   0.],
           [  0.,  40.,   0.,   0.,   0.,   0.],
           [  0.,   0.,  20.,   0.,   0.,   0.],
           [  0.,   0.,   0.,   3.,   0.,   0.],
           [  0.,   0.,   0.,   0.,   3.,   0.],
           [  0.,   0.,   0.,   0.,   0.,   3.]])
    
    """
    (x, y, z) = half_extents
    Ix = mass/3.*(y**2+z**2)
    Iy = mass/3.*(x**2+z**2)
    Iz = mass/3.*(x**2+y**2)
    return diag( (Ix, Iy, Iz, mass, mass, mass) )


def ellipsoid(radii, mass):
    """Mass matrix of an homogeneous ellipsoid.
    Dimensions are expressed at x,y,z coordinates
    
    >>> ellipsoid((3.,1.,2.), 5.)
    array([[  5.,   0.,   0.,   0.,   0.,   0.],
           [  0.,  13.,   0.,   0.,   0.,   0.],
           [  0.,   0.,  10.,   0.,   0.,   0.],
           [  0.,   0.,   0.,   5.,   0.,   0.],
           [  0.,   0.,   0.,   0.,   5.,   0.],
           [  0.,   0.,   0.,   0.,   0.,   5.]])
     
    """
    (x, y, z) = radii
    Ix = mass/5.*(y**2+z**2)
    Iy = mass/5.*(x**2+z**2)
    Iz = mass/5.*(x**2+y**2)
    return  diag( (Ix, Iy, Iz, mass, mass, mass) )
    
def cylinder(length, radius, mass):
    """Mass matrix of an homogeneous cylinder, whose symmetry axis is along the z-axis.
    
    >>> cylinder(1., 0.1, 12.)
    array([[  1.03,   0.  ,   0.  ,   0.  ,   0.  ,   0.  ],
           [  0.  ,   1.03,   0.  ,   0.  ,   0.  ,   0.  ],
           [  0.  ,   0.  ,   0.06,   0.  ,   0.  ,   0.  ],
           [  0.  ,   0.  ,   0.  ,  12.  ,   0.  ,   0.  ],
           [  0.  ,   0.  ,   0.  ,   0.  ,  12.  ,   0.  ],
           [  0.  ,   0.  ,   0.  ,   0.  ,   0.  ,  12.  ]])

    """
    L = length
    R = radius
    Itan = mass*(R**2/4. + L**2/12.)
    Iaxe = mass*R**2/2.
    return diag( (Itan, Itan, Iaxe, mass, mass, mass) )    

def sphere(radius, mass):
    """Mass matrix of an homogeneous sphere.
    
    >>> sphere(1., 5.)
    array([[ 2.,  0.,  0.,  0.,  0.,  0.],
           [ 0.,  2.,  0.,  0.,  0.,  0.],
           [ 0.,  0.,  2.,  0.,  0.,  0.],
           [ 0.,  0.,  0.,  5.,  0.,  0.],
           [ 0.,  0.,  0.,  0.,  5.,  0.],
           [ 0.,  0.,  0.,  0.,  0.,  5.]])
    
    """
    I = 2.*mass*radius**2/5.
    return diag( (I, I, I, mass, mass, mass) )

