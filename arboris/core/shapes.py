# coding=utf-8
"""
Frames...

TODO: add ellipse and pill shapes
"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from arboris import Shape

class Point(Shape):
    """
    A point
    """
    def __init__(self, frame):
        Shape.__init__(self, frame)


class Box(Shape):
    """
    A box
    """
    def __init__(self, frame, lengths=(1.,1.,1.)):
        Shape.__init__(self, frame)
        self.lengths = lengths


class Cylinder(Shape):
    """
    A cylinder
    """
    def __init__(self, frame, length=1., radius=1.):
        Shape.__init__(self, frame)
        self.radius = radius
        self.length = length


class Sphere(Shape):
    """
    A sphere
    """
    def __init__(self, frame, radius=1.):
        Shape.__init__(self, frame)
        self.radius = radius

