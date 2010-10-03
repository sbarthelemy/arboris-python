import arboristest
from numpy import array, zeros, eye, sin, cos, dot, ndarray
from numpy.linalg import norm
from arboris.core import World, Body
from arboris.joints import *
import arboris.homogeneousmatrix
from arboris.homogeneousmatrix import rotx

class TestRzyxAgainstRzRyRx(arboristest.TestCase):
    def setUp(self):
        """
        Ensure multi-dof Rzyx behaves like the combination of Rz, Ry, Rx.
        """
        world = World()
        self.Ba = Body()
        Rzyx = RzRyRxJoint()
        world.add_link(world.ground, Rzyx, self.Ba)
        Bzy = Body(name='zy')
        Byx = Body(name='yx')
        self.Bb = Body()
        Rz = RzJoint()
        world.add_link(world.ground, Rz, Bzy)
        Ry = RyJoint()
        world.add_link(Bzy, Ry, Byx)
        Rx = RxJoint()
        world.add_link(Byx, Rx, self.Bb)
        world.init()
        (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
        Rzyx.gpos[:] = (az, ay, ax)
        (Rz.gpos[0], Ry.gpos[0], Rx.gpos[0]) = (az, ay, ax)
        world.update_dynamic()

    def test_jacobian(self):
        Ja = self.Ba.jacobian[:,0:3]
        Jb = self.Bb.jacobian[:,3:6]
        self.assertListsAlmostEqual(Ja,Jb)

if __name__ == '__main__':
    arboristest.main()
