# coding=utf-8

"""
This module builds worlds composed of a single shape.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.joints import FreeJoint
from arboris.shapes import Sphere, Box, Cylinder
from arboris.core import World, Body, SubFrame
from arboris.homogeneousmatrix import transl
import arboris.massmatrix as massmatrix
import numpy

def add_sphere(world, radius=1., mass=1., name=None):
    """Build a ball robot.

    Example:

        >>> w = World()
        >>> add_sphere(w)
        >>> w.update_dynamic()

    """
    assert isinstance(world, World)
    ball = Body(
        name=name,
        mass=massmatrix.sphere(radius, mass))
    world.add_link(world.ground, FreeJoint(), ball)
    world.register(Sphere(ball, radius))
    world.init()
 

def add_box(world, lengths=(1.,1.,1.), mass=1., name='Box'):
    """Build a box robot.

    Example:

        >>> w = World()
        >>> add_box(w)
        >>> w.update_dynamic()

    """
    assert isinstance(world, World)
    box = Body(
        name=name,
        mass=massmatrix.box(lengths, mass))
    world.add_link(world.ground, FreeJoint(), box)
    world.register(Box(box, lengths))
    world.init()


def add_cylinder(world, length=1., radius=1., mass=1., name='Cylinder'):
    """Build a cylinder robot, whose symmetry axis is along the z-axis.

    Example:

        >>> w = World()
        >>> add_cylinder(w)
        >>> w.update_dynamic()

    """
    assert isinstance(world, World)
    cylinder = Body(
        name=name,
        mass=massmatrix.cylinder(length, radius, mass))
    world.add_link(world.ground, FreeJoint(), cylinder)
    world.register(Cylinder(cylinder, length, radius))
    world.init()
    
    
def add_groundplane(w, lengths=(1., .1, 1.) ):
    """Add a ground plane using a box shape.
    """
    frame = SubFrame(w.ground, transl(0., -lengths[1]/2., 0.) )
    w.register(Box(frame, lengths, 'Ground shape'))
