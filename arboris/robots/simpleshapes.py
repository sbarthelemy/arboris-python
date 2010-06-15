# coding=utf-8

"""
This module builds worlds composed of a single shape.
"""
__author__ = (u"Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.joints import FreeJoint
from arboris.shapes import Sphere, Box, Cylinder, Plane
from arboris.core import World, Body, SubFrame
from arboris.homogeneousmatrix import transl
import arboris.massmatrix as massmatrix

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


def add_box(world, half_extents=(1., 1., 1.), mass=1., name='Box'):
    """Build a box robot.

    Example:

        >>> w = World()
        >>> add_box(w)
        >>> w.update_dynamic()

    """
    assert isinstance(world, World)
    box = Body(
        name=name,
        mass=massmatrix.box(half_extents, mass))
    world.add_link(world.ground, FreeJoint(), box)
    world.register(Box(box, half_extents))
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


def add_groundplane(world):
    """Add a ground plane using a plane shape.
    """
    coeffs = list(world.up)
    coeffs.append(0.)
    world.register(Plane(world.ground, coeffs, 'Ground shape'))
