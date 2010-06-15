import unittest
import arboristest
from arboris.all import *

from numpy import arange, diag, sqrt

class PdControllerTestCase(arboristest.TestCase):

    def runTest(self):
        world = World()
        add_simplearm(world)
        joints = world.getjoints()
        gpos_des = (3.14/4, 3.14/4, 3.14/4)
        kp = 7*diag((1.,1.,1.))
        c = controllers.ProportionalDerivativeController(joints,
            gpos_des=gpos_des,
            kp=kp,
            kd=kp/sqrt(2))
        world.register(c)
        time = arange(0, 3, 1e-3)
        simulate(world, time)
        gpos = []
        for j in joints:
            gpos.extend(j.gpos)
        self.assertListsAlmostEqual(gpos, gpos_des, 1)

if __name__ == '__main__':
    unittest.main()
