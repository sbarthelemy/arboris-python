# coding=utf-8
"""Geometric shapes, for use in collision detection."""

__author__ = (u"Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from arboris.core import Shape
import numpy
import numpy.linalg

class Plane(Shape):
    """
    A plane.

    A plane defined by the equation `a x + b y + c z + d = 0`, in the ``frame``
    coordinates.

    :param frame: the frame the shape is attached to
    :type frame: :class:`arboris.frame`
    :param coeffs: coefficients `(a, b, c, d)` for the plane equation
    :type coeffs: (4,) array of floats
    :param name: the shape's name
    :type name: string

    """
    def __init__(self, frame, coeffs=(0., 1., 0., 0.), name=None):
        Shape.__init__(self, frame, name)
        # normalize the plane normal
        coeffs = numpy.array(coeffs)
        n = numpy.linalg.norm(coeffs[0:3])
        self.coeffs = coeffs/n

class Point(Shape):
    """
    A point.
    """
    def __init__(self, frame, name=None):
        Shape.__init__(self, frame, name)


class Box(Shape):
    """
    A box.
    """
    def __init__(self, frame, half_extents=(1., 1., 1.), name=None):
        Shape.__init__(self, frame, name)
        self.half_extents = half_extents


class Cylinder(Shape):
    """
    A cylinder, whose symmetry axis is along the z-axis.
    """
    def __init__(self, frame, length=1., radius=1., name=None):
        assert radius >= 0.
        Shape.__init__(self, frame, name)
        self.radius = radius
        self.length = length


class Sphere(Shape):
    """
    A sphere.
    """
    def __init__(self, frame, radius=1., name=None):
        assert radius >= 0.
        Shape.__init__(self, frame, name)
        self.radius = radius

