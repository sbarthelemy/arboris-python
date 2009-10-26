# coding=utf-8
"""
Build a planar snake robot.

This module builds a planar snake robot.

When ran as a script, the module shows the robot in motion.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.core import World, Body, SubFrame
from arboris.massmatrix import transport, cylinder, box
from arboris.homogeneousmatrix import transl, rotx
from arboris.joints import FreeJoint, RzJoint
from numpy import array, dot, pi

def add_snake(w, nbody, lengths = None, masses = None, gpos = None, gvel = None, is_fixed = True):
    """Add a snake-shape robot to the world.
    
    **Example:**
    
        >>> w = World()
        >>> add_snake(w, 3, lengths=[1.5, 1., 5.])
    
    """
    assert isinstance(w, World)
    if lengths is None:
        lengths = [.5] * nbody
    assert nbody == len(lengths)
    if masses is None:
        masses = [2.] * nbody
    assert nbody == len(masses)
    if gpos is None:
        gpos = [0.] * nbody
    assert nbody == len(gpos)
    if gvel is None:
        gvel = [0.] * nbody
    assert nbody == len(gvel)
    
    if is_fixed:
        frame = w.ground
    else:
        L = lengths[0]/2.
        body = Body(mass=box([L, L, L], masses[0]))
        w.add_link(w.ground, FreeJoint(), body)
        frame = body

    for (length, mass, q, dq) in zip(lengths, masses, gpos, gvel):
        radius = length/10. #TODO use 5 instead of 10
        #angle = pi/2.
        angle = 0. #TODO: remove, it is here for testing
        body = Body(mass=transport(cylinder(length, radius, mass),
                                   dot(rotx(angle),
                                       transl(0., -length/2., 0.))))
        joint = RzJoint(gpos=q, gvel=dq)
        w.add_link(frame, joint, body)
        frame = SubFrame(body, transl(0., length, 0.))
    w.register(frame)
    w.init()
    
    
