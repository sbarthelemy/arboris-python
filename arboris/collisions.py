# coding=utf-8
"""
Collision solvers...

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy.linalg import norm
from numpy import zeros, eye, dot, absolute, argsort, cross
proximity = 0.01

def sphere_sphere_collision(shapes):
    """

    svg:

    .. image:: img/sphere_sphere_collision.png
    

    """    
    return _sphere_sphere_collision(shapes[0].frame.pose, 
                                    shapes[1].frame.pose,
                                    shapes[0].radius, 
                                    shapes[1].radius)


def _sphere_sphere_collision(p_g0, p_g1, radius0, radius1):
    """
    Tests:

    >>> from numpy import array, zeros
    >>> p_g0 = zeros((3))
    >>> p_g1 = array( (2.,2.,1.) )
    >>> (sdist, H_gc0, H_gc1) = _sphere_sphere_collision(p_g0, p_g1, 1.1, 1.2)
    >>> print(sdist)
    0.7
    >>> print(H_gc0)
    [[ 0.70710678  0.23570226  0.66666667  0.73333333]
     [-0.70710678  0.23570226  0.66666667  0.73333333]
     [ 0.         -0.94280904  0.33333333  0.36666667]
     [ 0.          0.          0.          1.        ]]
    >>> print(H_gc1)
    [[ 0.70710678  0.23570226  0.66666667  1.2       ]
     [-0.70710678  0.23570226  0.66666667  1.2       ]
     [ 0.         -0.94280904  0.33333333  0.6       ]
     [ 0.          0.          0.          1.        ]]
    >>> (sdist, H_gc0, H_gc1) = _sphere_sphere_collision(p_g0, p_g1, 1.5, 1.6)
    >>> print(sdist)
    -0.1
    >>> print(H_gc0)
    [[ 0.70710678  0.23570226  0.66666667  1.        ]
     [-0.70710678  0.23570226  0.66666667  1.        ]
     [ 0.         -0.94280904  0.33333333  0.5       ]
     [ 0.          0.          0.          1.        ]]
    >>> print(H_gc1)
    [[ 0.70710678  0.23570226  0.66666667  0.93333333]
     [-0.70710678  0.23570226  0.66666667  0.93333333]
     [ 0.         -0.94280904  0.33333333  0.46666667]
     [ 0.          0.          0.          1.        ]]

    """
    vec = p_g1 - p_g0
    sdist = norm(vec) - radius0 - radius1
    H_gc0 = eye(4)
    x = H_gc0[0:3,0]
    y = H_gc0[0:3,1]
    z = H_gc0[0:3,2]
    p = H_gc0[0:3,3]
    # z-axis, normal to the tangeant plane:
    z[:] = vec/norm(vec)
    idx = argsort(absolute(z))
    # x axis, normal to z-axis
    x[idx[0]] = 0
    x[idx[1]] = z[idx[2]]
    x[idx[2]] = -z[idx[1]]
    x /= norm(x)
    y[:] = cross(z, x)
    p[:] = p_g0 + radius0*z
    H_gc1 = H_gc0.copy()
    H_gc1[0:3,3] += sdist*z
    return (sdist, H_gc0, H_gc1)

def point_box_collision():
    pass

