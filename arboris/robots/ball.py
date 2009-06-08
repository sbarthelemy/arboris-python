# coding=utf-8

"""
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.joints import FreeJoint
from arboris.shapes import Sphere, Box, Cylinder
from arboris.core import World, Body
import numpy

def mass_parallelepiped(m,lengths):
    """The mass matrix of an homogeneous parallelepiped."""
    (a, b, c) = lengths
    return numpy.diag((
        m/12.*(b**2+c**2 ), 
        m/12.*(a**2+c**2), 
        m/12.*(a**2+b**2),
        m, m, m))

def ball(world=None, radius=1., mass=1., name=None):
    """Build a ball robot.
    TODO: fix inertia

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
        mass=mass_parallelepiped(mass, (radius,radius,radius)))
    freejoint = FreeJoint()
    freejoint.attach(w.ground, ball)
    w.register(ball)
    s = Sphere(ball, radius)
    w.register(freejoint)
    w.initjointspace()
    return w
 

def box(world=None, lengths=(1.,1.,1.), mass=1., name='Box'):
    """Build a box robot..
    TODO: fix inertia
    
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
        mass=mass_parallelepiped(mass, (lengths[0], lengths[1], lengths[2])))
    freejoint = FreeJoint()
    freejoint.attach(w.ground, box)
    w.register(box)
    s = Box(box, lengths)
    w.register(freejoint)
    w.initjointspace()
    return w


def cylinder(world=None, radius=1., length=1., mass=1., name='Cylinder'):
    """Build a cylinder robot.
    TODO: fix inertia

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
        mass=mass_parallelepiped(mass, (length, length, length)))
    freejoint = FreeJoint(frames=(w.ground, cylinder))
    w.register(cylinder)
    s = Cylinder(cylinder, length, radius)
    w.register(s)
    w.initjointspace()
    return w
    
