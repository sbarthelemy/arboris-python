# coding=utf-8

"""
This module builds worlds composed of a single shape.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.joints import FreeJoint
from arboris.shapes import Sphere, Box, Cylinder
from arboris.core import World, Body
import arboris.massmatrix as massmatrix
import numpy

def ball(world=None, radius=1., mass=1., name=None):
    """Build a ball robot.

    Example:

        >>> r = ball()
        >>> r.update_dynamic()

    """

    # Create a world
    if world is None:
        w = World()
    elif isinstance(world, World):
        w = world
    else:
        raise ValueError('the world argument must be an instance of the World class')

    ball = Body(
        name=name,
        mass=massmatrix.sphere(radius, mass))
    freejoint = FreeJoint(frames=(w.ground, ball))
    s = Sphere(ball, radius)
    w.register(freejoint)
    w.register(s)
    w.init()
    return w
 

def box(world=None, lengths=(1.,1.,1.), mass=1., name='Box'):
    """Build a box robot.

    Example:

        >>> r = box()
        >>> r.update_dynamic()

    """

    # Create a world
    if world is None:
        w = World()
    elif isinstance(world, World):
        w = world
    else:
        raise ValueError('the world argument must be an instance of the World class')
    
    box = Body(
        name=name,
        mass=massmatrix.box(lengths, mass))
    freejoint = FreeJoint()
    freejoint.attach(w.ground, box)
    w.register(box)
    s = Box(box, lengths)
    w.register(freejoint)
    w.init()
    return w


def cylinder(world=None, length=1., radius=1., mass=1., name='Cylinder'):
    """Build a cylinder robot, whose symmetry axis is along the z-axis.

    Example:

        >>> r = cylinder()
        >>> r.update_dynamic()

    """

    # Create a world
    if world is None:
        w = World()
    elif isinstance(world, World):
        w = world
    else:
        raise ValueError('the world argument must be an instance of the World class')

    cylinder = Body(
        name=name,
        mass=massmatrix.cylinder(length, radius, mass))
    freejoint = FreeJoint(frames=(w.ground, cylinder))
    w.register(cylinder)
    s = Cylinder(cylinder, length, radius)
    w.register(s)
    w.init()
    return w
    
