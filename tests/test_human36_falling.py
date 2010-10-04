import arboristest
from arboris.all import *
import numpy

class Human36FallingTestCase(arboristest.TestCase):

    def setUp(self):
        d = numpy
        t_end = 20e-2
        dt = 5e-3
        self.world = World()
        observers = []
        add_groundplane(self.world)
        add_human36(self.world)
        # set initial position
        self.world.ground.childrenjoints[0].gpos = numpy.dot(
                homogeneousmatrix.transl(0,0.03,0),
                self.world.ground.childrenjoints[0].gpos)
        # add weight
        self.world.register(controllers.WeightController())

        # add a (single) PD Controller to all the simple joints
        ljoints = filter(
                lambda x: isinstance(x, core.LinearConfigurationSpaceJoint),
                self.world.iterjoints());
        n = len(JointsList(ljoints).dof)
        kp = 0.1*numpy.eye(n)
        c = ProportionalDerivativeController(ljoints, kp=kp, kd=2.*numpy.sqrt(kp))
        # add contacts
        shapes = self.world.getshapes()
        self.contact_frames = []
        for c in constraints.get_all_contacts(self.world, friction_coeff=.6):
            self.world.register(c)
            # find the contact frame attached to the foot (and not the ground)
            if c._frames[0].body.name.startswith('Foot'):
                self.contact_frames.append(c._frames[0])
            else:
                self.contact_frames.append(c._frames[1])
        # run the simulation
        timeline = arange(0., t_end, dt)
        simulate(self.world, timeline, observers)

    def runTest(self):
        # ensure contact points are still above the ground
        for f in self.contact_frames:
            self.assertTrue(f.pose[1,3] >= 0.)

if __name__ == '__main__':
    arboristest.main()
