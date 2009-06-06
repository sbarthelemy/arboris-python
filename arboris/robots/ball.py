# coding=utf-8

"""
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.core.joints import FreeJoint
from arboris.core.shapes import Sphere, Box, Cylinder
from arboris import World, Body
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
    return w
    
def box(world=None, lengths=(1.,1.,1.), mass=1., name='Box'):
    """Build a box robot..
    TODO: fix inertia
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
    return w

def cylinder(world=None, radius=1., length=1., mass=1., name='Cylinder'):
    """Build a cylinder robot.
    TODO: fix inertia
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
    return w
    
if __name__ == "__main__":

    w = ball()
    
    #compute dynamic model
    w.update_dynamic()

