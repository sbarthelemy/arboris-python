# coding=utf-8
"""Collision solvers...

"""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from numpy.linalg import norm
from numpy import zeros, eye, dot, absolute, argsort, cross, argmin,\
    hstack
import homogeneousmatrix as Hg
from core import Shape
from shapes import *

def choose_solver(shape0, shape1):
    """Choose a suitable solver for the two shapes.

    """
    assert isinstance(shape0, Shape)
    assert isinstance(shape1, Shape)

    if isinstance(shape0, Sphere):
        if isinstance(shape1, Sphere):
            shapes = (shape0, shape1)
            solver = sphere_sphere_collision
        elif isinstance(shape1, Point):
            shapes = (shape0, shape1)
            solver = sphere_point_collision
        elif isinstance(shape1, Box):
            shapes = (shape1, shape0)
            solver = box_sphere_collision
        else:
            raise NotImplementedError()
    elif isinstance(shape0, Point):
        if isinstance(shape1, Sphere):
            shapes = (shape1, shape0)
            solver = sphere_point_collision
        elif isinstance(shape1, Box):
            shapes = (shape1, shape0)
            solver = box_point_collision
        else:
            raise NotImplementedError()
    elif isinstance(shape0, Box):
        if isinstance(shape1, Sphere):
            shapes = (shape0, shape1)
            solver = box_sphere_collision
        elif isinstance(shape1, Point):
            shapes = (shape0, shape1)
            solver = box_point_collision
        else:
            raise NotImplementedError()
    else:
        raise NotImplementedError()
    return (shapes, solver)


def _normal_to_frame(vec):
    """Builds a direct frame whose z-axis is vec.

    :param vec: input 
    :type vec: (3,) array
    :return: homogeneous matrix of the frame
    :rtype: (4,4) array

    **Examples:**
    >>> from numpy import array
    >>> _normal_to_frame(array([1.,0.,0.]))
    array([[-0.,  0.,  1.,  0.],
           [ 0., -1.,  0.,  0.],
           [ 1.,  0.,  0.,  0.],
           [ 0.,  0.,  0.,  1.]])

    """
    assert abs(norm(vec)-1) < 1e-9
    H = eye(4)
    x = H[0:3,0]
    y = H[0:3,1]
    z = H[0:3,2]
    # z-axis, normal to the tangent plane:
    z[:] = vec
    idx = argsort(absolute(z))
    # x axis, normal to z-axis
    x[idx[0]] = 0
    x[idx[1]] = z[idx[2]]
    x[idx[2]] = -z[idx[1]]
    x /= norm(x)
    y[:] = cross(z, x)
    return H


def sphere_sphere_collision(shapes):
    """

    """
    assert isinstance(shapes[0], Sphere)
    assert isinstance(shapes[1], Sphere)
    return _sphere_sphere_collision(shapes[0].frame.pose[0:3,3], 
                                    shapes[0].radius,
                                    shapes[1].frame.pose[0:3,3],
                                    shapes[1].radius)

def sphere_point_collision(shapes):
    """

    """
    assert isinstance(shapes[0], Sphere)
    assert isinstance(shapes[1], Point)
    return _sphere_sphere_collision(shapes[0].frame.pose[0:3,3], 
                                    shapes[0].radius,
                                    shapes[1].frame.pose[0:3,3],
                                    0.)

def box_sphere_collision(shapes):
    """


    """
    assert isinstance(shapes[0], Box)
    assert isinstance(shapes[1], Sphere)
    return _box_sphere_collision(shapes[0].frame.pose, 
                                    shapes[0].half_extents,
                                    shapes[1].frame.pose[0:3,3],
                                    shapes[1].radius)

def box_point_collision(shapes):
    """

    """
    assert isinstance(shapes[0], Box)
    assert isinstance(shapes[1], Point)
    return _box_sphere_collision(shapes[0].frame.pose, 
                                    shapes[0].half_extents,
                                    shapes[1].frame.pose[0:3,3],
                                    0.)

def _sphere_sphere_collision(p_g0, radius0, p_g1, radius1):
    """

    .. image:: img/sphere_sphere_collision.png

    **Tests:**

    >>> from numpy import array, zeros
    >>> p_g0 = zeros((3))
    >>> p_g1 = array( (2.,2.,1.) )
    >>> (sdist, H_gc0, H_gc1) = _sphere_sphere_collision(p_g0, 1.1, p_g1, 1.2)
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
    >>> (sdist, H_gc0, H_gc1) = _sphere_sphere_collision(p_g0, 1.5, p_g1, 1.6)
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
    normal = vec/norm(vec)
    H_gc0 = _normal_to_frame(normal)
    z = H_gc0[0:3,2]
    H_gc0[0:3,3] = p_g0 + radius0*z
    H_gc1 = H_gc0.copy()
    H_gc1[0:3,3] += sdist*z
    return (sdist, H_gc0, H_gc1)



def _box_sphere_collision(H_g0, half_extents0, p_g1, radius1):
    """
    :param H_g0: pose of the box `H_{g1}`
    :type H_g0: (4,4) array
    :param lengths0: lengths of the box
    :type lengths0: (3,) array
    :param p_g1: center of the sphere
    :type p_g1: (3,) array 
    :param radius1: radius of the sphere
    :type radius1: float

    .. image:: img/box_sphere_collision.png

    **Tests:**
    
    >>> from numpy import array, eye
    >>> H_g0 = eye(4)
    >>> lengths0 = array([1., 2., 3.])
    >>> r1 = 0.1
    >>> p_g1 = array([0., 3., 1.])
    >>> (sdist, H_gc0, H_gc1) = _box_sphere_collision(H_g0, lengths0/2, p_g1, r1)
    >>> print(sdist)
    1.9
    >>> print(H_gc0)
    [[ 0.  1.  0.  0.]
     [-0.  0.  1.  1.]
     [ 1. -0.  0.  1.]
     [ 0.  0.  0.  1.]]
    >>> print(H_gc1)
    [[ 0.   1.   0.   0. ]
     [-0.   0.   1.   2.9]
     [ 1.  -0.   0.   1. ]
     [ 0.   0.   0.   1. ]]
    >>> p_g1 = array([0.55, 0., 0.])
    >>> (sdist, H_gc0, H_gc1) = _box_sphere_collision(H_g0, lengths0/2, p_g1, r1)
    >>> print(sdist)
    -0.05
    >>> print(H_gc0)
    [[-0.   0.   1.   0.5]
     [ 0.  -1.   0.   0. ]
     [ 1.   0.   0.   0. ]
     [ 0.   0.   0.   1. ]]
    >>> print(H_gc1)
    [[-0.    0.    1.    0.45]
     [ 0.   -1.    0.    0.  ]
     [ 1.    0.    0.    0.  ]
     [ 0.    0.    0.    1.  ]]
    >>> p_g1 = array([0.45, 0., 0.])
    >>> (sdist, H_gc0, H_gc1) = _box_sphere_collision(H_g0, lengths0/2, p_g1, r1)
    >>> print(sdist)
    -0.15
    >>> print(H_gc0)
    [[-0.   0.   1.   0.5]
     [ 0.  -1.   0.   0. ]
     [ 1.   0.   0.   0. ]
     [ 0.   0.   0.   1. ]]
    >>> print(H_gc1)
    [[-0.    0.    1.    0.35]
     [ 0.   -1.    0.    0.  ]
     [ 1.    0.    0.    0.  ]
     [ 0.    0.    0.    1.  ]]

    """
    assert Hg.ishomogeneousmatrix(H_g0)
    p_01 = Hg.pdot(Hg.inv(H_g0), p_g1)
    if (abs(p_01) <= half_extents0).all():
        # p_01 is inside the box, we need to find the nearest face
        i = argmin(hstack((half_extents0 - p_01, half_extents0 + p_01)))
        f_0 = p_01.copy()
        normal = zeros(3)
        if i < 3:
            f_0[i] = half_extents0[i]
            normal[i] = 1
        else:
            f_0[i-3] = -half_extents0[i-3]
            normal[i-3] = -1 #TODO check this line is correct
        f_g = Hg.pdot(H_g0, f_0)
        sdist = -norm(f_g - p_g1)-radius1
    else:
        # find the point x inside the box that is the nearest to 
        # the sphere center:
        f_0 = zeros(3)
        for i in range(3):
            f_0[i] = max(min(half_extents0[i], p_01[i]), -half_extents0[i])
        f_g = Hg.pdot(H_g0, f_0)
        vec = p_g1 - f_g
        normal = vec/norm(vec)
        sdist = norm(vec) - radius1
    H_gc0 = _normal_to_frame(normal)
    H_gc1 = H_gc0.copy()
    H_gc0[0:3,3] = f_g
    H_gc1[0:3,3] = p_g1 - radius1*normal
    return (sdist, H_gc0, H_gc1)



