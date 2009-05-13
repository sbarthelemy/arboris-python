# coding=utf-8

"""
Build a planar 3-R robot.

This module builds a planar 3-r robot, identical to the one built in the
t06_ConstructionRobot.m tutorial from matlab-arboris.

When ran as a script, the module shows the robot in motion.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from arboris import World, Body, SubFrame
import numpy as np
import homogeneousmatrix as Hg
from joints import RzJoint

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

def triplehinge(world=None):
    """Build a  planar 3-R robot."""

    arm_length=0.5
    arm_mass=1
    forearm_length=0.4
    forearm_mass=0.8
    hand_length=0.2
    hand_mass=0.2

    # Create a world
    if world is None:
        w = World()
    elif isinstance(world, World):
        w = world
    else:
        raise ValueError('the world argument must be an instance of the World class')

    # create other bodies
    arm = Body(
        name='Arm',
        mass=transport_mass_matrix(
            mass_parallelepiped(
                arm_mass, 
                (arm_length/10,arm_length,arm_length/10)),
                Hg.transl((0,arm_length/2,0))))
    forearm = Body(
        name='ForeArm',
        mass=transport_mass_matrix(
            mass_parallelepiped(
                forearm_mass, 
                (forearm_length/10,forearm_length,forearm_length/10)),
                Hg.transl((0,forearm_length/2,0))))
    hand = Body(
        name='Hand',
        mass=transport_mass_matrix(
            mass_parallelepiped(
                hand_mass, 
                (hand_length/10,hand_length,hand_length/10)),
                Hg.transl((0,hand_length/2,0))))


    # create a joint between the ground and the arm
    shoulder = RzJoint(name='Shoulder')
    # add the new joint to the world (this will also add arm to w.bodies)
    w.add_joint(joint=shoulder, frames=(w.ground, arm) )
    
    # add a frame to the arm, where the forearm will be anchored
    f = SubFrame(arm,
        Hg.transl((0,arm_length,0)),
        'ElbowLeftFrame')
    # create a joint between the arm and the forearm
    elbow = RzJoint(name='Elbow')
    w.add_joint(joint=elbow, frames=(f, forearm) )

    # add a frame to the forearm, where the hand will be anchored
    f = SubFrame(forearm,
        Hg.transl((0,forearm_length,0)),
        'WristLeftFrame')
    # create a joint between the forearm and the hand
    wrist = RzJoint(name = 'Wrist')
    w.add_joint( wrist, (f, hand) )

    return w

if __name__ == "__main__":

    w = triplehinge()
    
    #compute dynamic model
    w.update_dynamic()

