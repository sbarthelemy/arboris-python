# coding=utf-8

"""
Build a planar 3-R robot.

This module builds a planar 3-r robot, identical to the one built in the
t06_ConstructionRobot.m tutorial from matlab-arboris.

When ran as a script, the module shows the robot in motion.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris.core import World, Body, SubFrame
import numpy as np
import arboris.homogeneousmatrix as Hg
from arboris.joints import RzJoint

def transport_mass_matrix(mass,H):
    """Transport (express) the mass matrix into another frame."""
    Ad = Hg.iadjoint(H)
    return np.dot(
        Ad.transpose(),
        np.dot(mass, Ad))
        
def mass_parallelepiped(m,lengths):
    """The mass matrix of an homogeneous parallelepiped."""
    (a, b, c) = lengths
    return np.diag((
        m/12.*(b**2+c**2 ), 
        m/12.*(a**2+c**2), 
        m/12.*(a**2+b**2),
        m, m, m))

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
        mass=transport_mass_matrix(
            mass_parallelepiped(
                arm_mass, 
                (arm_length/10,arm_length,arm_length/10)),
                Hg.transl(0,arm_length/2,0)))
    forearm = Body(
        name=name+'ForeArm',
        mass=transport_mass_matrix(
            mass_parallelepiped(
                forearm_mass, 
                (forearm_length/10,forearm_length,forearm_length/10)),
                Hg.transl(0,forearm_length/2,0)))
    hand = Body(
        name=name+'Hand',
        mass=transport_mass_matrix(
            mass_parallelepiped(
                hand_mass, 
                (hand_length/10,hand_length,hand_length/10)),
                Hg.transl(0,hand_length/2,0)))


    # create a joint between the ground and the arm
    shoulder = RzJoint(name=name+'Shoulder', frames=(w.ground, arm))

    # add the new joint to the world (this will also add arm to w)
    w.register(shoulder)
    
    # add a frame to the arm, where the forearm will be anchored
    f = SubFrame(arm,
        Hg.transl(0,arm_length,0),
        name+'ElbowBaseFrame')

    # create a joint between the arm and the forearm
    elbow = RzJoint(name=name+'Elbow', frames=(f, forearm))
    w.register(elbow)

    # add a frame to the forearm, where the hand will be anchored
    f = SubFrame(forearm,
        Hg.transl(0,forearm_length,0),
        name+'WristBaseFrame')

    # create a joint between the forearm and the hand
    wrist = RzJoint(name=name+'Wrist', frames=(f, hand))
    w.register(wrist)

    # create a frame at the end of the hand
    f = SubFrame(hand, Hg.transl(0,hand_length,0), name+'EndEffector')
    w.register(f)
    w.init()

