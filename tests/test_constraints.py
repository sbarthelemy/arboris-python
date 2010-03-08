# coding=utf-8

import unittest
from arboristest import TestCase
from numpy import arange, eye
from arboris.constraints import JointLimits, BallAndSocketConstraint
from arboris.controllers import WeightController
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint

class JointLimitsTestCase(TestCase):
    """Check if joint limits are enforced on a simplearm under grabity."""
    def setUp(self):
        self.world = simplearm()
        a = WeightController()
        self.world.register(a)
        joints = self.world.getjoints()
        self.shoulder = joints['Shoulder']
        c = JointLimits(self.shoulder, -3.14/2, 3.14/2)
        self.world.register(c)
   
    def test_max_joint_limit(self):
        self.shoulder.gpos[0] = 3.14/2 - 0.1
        time = arange(0., 0.1, 1e-3)
        simulate(self.world, time)
        self.assertTrue(3.14/2 >= self.shoulder.gpos[0])

    def test_min_joint_limit(self):
        self.shoulder.gpos[0] = -3.14/2 + 0.1
        time = arange(0., 0.1, 1e-3)
        simulate(self.world, time)
        self.assertTrue(-3.14/2 <= self.shoulder.gpos[0])

class BallAndSocketTestCase(TestCase):
    """
    A body under gravity, fixed to the ground via a ball and socket constraint.
    """
    
    def runTest(self):
        b0 = Body(mass=eye(6))
        w = World()
        w.add_link(w.ground, FreeJoint(), b0)
        w.init()
        ctrl =  WeightController()
        w.register(ctrl)
        c0 = BallAndSocketConstraint(frames=(w.ground, b0))
        w.register(c0)
        w.init()
        w.update_dynamic()
        dt = 0.001
        w.update_controllers(dt)
        w.update_constraints(dt)
        self.assertListsAlmostEqual(c0._force, [ 0.  ,  9.81,  0.  ])
        w.integrate(dt)
        w.update_dynamic()
        self.assertListsAlmostEqual(b0.pose,
            [[1.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])

if __name__ == '__main__':
    unittest.main()
