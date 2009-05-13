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
    def __init__(self, shape):
        Shape.__init__(self, shape)


class Box(Shape):
    """
    A box
    """
    def __init__(self, shape, dims=(1.,1.,1.)):
        Shape.__init__(self, shape)
        self.dims = dims


class Sphere(Shape):
    """
    A sphere
    """
    def __init__(self, shape, radius=1.):
        Shape.__init__(self, shape)
        self.radius = radius


if __name__ == "__main__":
    import doctest
    doctest.testmod()
