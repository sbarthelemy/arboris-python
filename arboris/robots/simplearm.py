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
from arboris.joints import RzJoint

def add_simplearm(world, name='', lengths=(0.5 ,0.4 , 0.2),
                  masses=(1.0, 0.8, 0.2)):
    """Build a  planar 3-R robot.

    TODO: make use of the ``name`` input argument to prefix bodies and joints names

    Example:

        >>> w = World()
        >>> add_simplearm(w)
        >>> w.update_dynamic()

    """
    assert isinstance(world, World)
    w = world
    arm_length = lengths[0]
    arm_mass = masses[0]
    forearm_length = lengths[1]
    forearm_mass = masses[1]
    hand_length = lengths[2]
    hand_mass = masses[2]

    # create bodies
    arm = Body(
        name=name+'Arm',
        mass=arboris.massmatrix.transport(
            arboris.massmatrix.box(
                (arm_length/20., arm_length/2., arm_length/20.),
                arm_mass),
            Hg.transl(0., -arm_length/2., 0.)))
    forearm = Body(
        name=name+'ForeArm',
        mass=arboris.massmatrix.transport(
            arboris.massmatrix.box(
                (forearm_length/20., forearm_length/2., forearm_length/20.),
                forearm_mass),
            Hg.transl(0., -forearm_length/2., 0.)))
    hand = Body(
        name=name+'Hand',
        mass=arboris.massmatrix.transport(
            arboris.massmatrix.box(
                (hand_length/20., -hand_length/2., hand_length/20.),
                hand_mass),
            Hg.transl(0., -hand_length/2., 0.)))

    # create a joint between the ground and the arm
    shoulder = RzJoint(name=name+'Shoulder')
    w.add_link(w.ground, shoulder, arm)
    
    # add a frame to the arm, where the forearm will be anchored
    f = SubFrame(arm,
        Hg.transl(0, arm_length, 0),
        name+'ElbowBaseFrame')

    # create a joint between the arm and the forearm
    elbow = RzJoint(name=name+'Elbow')
    w.add_link(f, elbow, forearm)

    # add a frame to the forearm, where the hand will be anchored
    f = SubFrame(forearm,
        Hg.transl(0, forearm_length, 0),
        name+'WristBaseFrame')

    # create a joint between the forearm and the hand
    wrist = RzJoint(name=name+'Wrist')
    w.add_link(f, wrist, hand)

    # create a frame at the end of the hand
    f = SubFrame(hand, Hg.transl(0, hand_length, 0), name+'EndEffector')
    w.register(f)
    w.init()

