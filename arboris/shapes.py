# coding=utf-8
"""Geometric shapes, for use in collision detection."""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from core import Shape

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
    def __init__(self, frame, half_extents=(1.,1.,1.), name=None):
        Shape.__init__(self, frame, name)
        self.half_extents = half_extents


class Cylinder(Shape):
    """
    A cylinder, whose symmetry axis is along the z-axis.
    """
    def __init__(self, frame, length=1., radius=1., name=None):
        Shape.__init__(self, frame, name)
        self.radius = radius
        self.length = length


class Sphere(Shape):
    """
    A sphere.
    """
    def __init__(self, frame, radius=1., name=None):
        Shape.__init__(self, frame, name)
        self.radius = radius

