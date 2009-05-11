# coding=utf-8
"""
Frames...

TODO: add ellipse and pill shapes
"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from arboris import Shape
from misc import NamedObject
from numpy.linalg import norm
from numpy import zeros, eye, dot, absolute, argsort, cross
import homogeneousmatrix as Hg
proximity = 0.01



class Point(Shape):
    """
    A point
    """
    def __init__(self, name=None):
        NamedObject.__init__(self, name)


class Box(Shape):
    """
    A box
    """
    def __init__(self, dims):
        self.dims = dims


class Sphere(Shape):
    """
    A box
    """
    def __init__(self, radius):
        self.radius = radius


class Collision(object):
    """
    A collision
    """

    def __init__(self, shapes, solver):
        self._shapes = shapes
        self._solver = solver

    def is_near(self):
        pass


def sphere_sphere_collision(H_g0, H_g1, radius0, radius1):
    """
    Examples:
    >>> from numpy import array
    >>> H_g0 = eye(4)
    >>> H_g1 = eye(4)
    >>> H_g1[0:3,3] = [2.,2.,1]
    >>> sphere_sphere_collision(H_g0, H_g1, 1.1, 1.2)
    >>> sphere_sphere_collision(H_g0, H_g1, 1.5, 1.6)
    (array([[ 0.70710678,  0.        ,  0.        ,  0.        ],
           [-0.70710678,  1.        ,  0.        ,  0.        ],
           [ 0.        ,  0.        ,  1.        ,  0.        ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]]), array([[ 0.70710678,  0.        ,  0.        ,  0.        ],
           [-0.70710678,  1.        ,  0.        ,  0.        ],
           [ 0.        ,  0.        ,  1.        , -0.1       ],
           [ 0.        ,  0.        ,  0.        ,  1.        ]]), -0.10000000000000009)

    """
    vec = H_g1[0:3,3] - H_g0[0:3,3]
    sdist = norm(vec) - radius0 - radius1
    if sdist < proximity:
        H_gc0 = eye(4)
        x = H_gc0[0:3,0]
        y = H_gc0[0:3,1]
        z = H_gc0[0:3,2]
        p = H_gc0[0:3,3]
        # z-axis, normal to the tangeant plane:
        z = vec/norm(vec)
        idx = argsort(absolute(z))
        # x axis, normal to z-axis
        x[idx[0]] = 0
        x[idx[1]] = z[idx[2]]
        x[idx[2]] = -z[idx[1]]
        x /= norm(x)
        y = cross(z, x)
        p = H_gc0[0:3,3] + radius0*z
        H_0c0 = dot(Hg.inv(H_g0), H_gc0)
        H_1c1 = H_0c0.copy()
        H_1c1[2,3] += sdist
        return (H_0c0, H_1c1, sdist)
    else:
        return None

def point_box_collision():
    pass


if __name__ == "__main__":
    import doctest
    doctest.testmod()

