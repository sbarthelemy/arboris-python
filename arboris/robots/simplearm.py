# coding=utf-8

"""
Build a planar 3-R robot.

This module builds a planar 3-r robot, identical to the one built in the
t06_ConstructionRobot.m tutorial from matlab-arboris.

When ran as a script, the module shows the robot in motion.
"""
__author__ = (u"Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.core import World, Body, SubFrame
import numpy as np
import arboris.homogeneousmatrix as Hg
import arboris.massmatrix
import arboris.shapes
from arboris.joints import RzJoint

def add_simplearm(world, name='', lengths=(0.5 ,0.4 , 0.2),
                  masses=(1.0, 0.8, 0.2), with_shapes=False):
    """Build a  planar 3-R robot.

    ***Example:***

    >>> w = World()
    >>> add_simplearm(w)
    >>> w.update_dynamic()

    """
    assert isinstance(world, World)

    def create_body(name, length, mass):
        """create a body"""
        half_extents = (length/20., length/2., length/20.)
        mass_matrix_at_com = arboris.massmatrix.box(half_extents, mass)
        mass_matrix_at_base = arboris.massmatrix.transport(
                mass_matrix_at_com,
                Hg.transl(0., -length/2., 0.))
        body = Body(name, mass_matrix_at_base)
        if with_shapes:
            f = SubFrame(body, Hg.transl(0., length/2., 0.))
            shape = arboris.shapes.Box(f, half_extents)
            world.register(shape)
        return body

    # create the 3 bodies
    arm = create_body(name+'Arm', lengths[0], masses[0])
    forearm = create_body(name+'ForeArm', lengths[1], masses[1])
    hand = create_body(name+'Hand', lengths[2], masses[2])

    # create a joint between the ground and the arm
    shoulder = RzJoint(name=name+'Shoulder')
    world.add_link(world.ground, shoulder, arm)

    # add a frame to the arm, where the forearm will be anchored
    f = SubFrame(arm,
        Hg.transl(0, lengths[0], 0),
        name+'ElbowBaseFrame')

    # create a joint between the arm and the forearm
    elbow = RzJoint(name=name+'Elbow')
    world.add_link(f, elbow, forearm)

    # add a frame to the forearm, where the hand will be anchored
    f = SubFrame(forearm,
        Hg.transl(0, lengths[1], 0),
        name+'WristBaseFrame')

    # create a joint between the forearm and the hand
    wrist = RzJoint(name=name+'Wrist')
    world.add_link(f, wrist, hand)

    # create a frame at the end of the hand
    f = SubFrame(hand, Hg.transl(0, lengths[2], 0), name+'EndEffector')
    world.register(f)
    world.init()

