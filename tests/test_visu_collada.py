# coding=utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from arboristest import TestCase
from arboris.all import *
import unittest
import tempfile
from os.path import join
tempdir = tempfile.gettempdir()

class VisuColladaTestCase(TestCase):

    def setUp(self):
        self.world = World()
        self.world.register(WeightController())
        add_simplearm(self.world, with_shapes=True)
        self.world.getjoints()['Shoulder'].gpos[0] = 3.14/4

    def test_animation(self):
        filename = join(tempdir, 'simplearm_simulation.h5')
        simulate(self.world, arange(0, 1, .01),
                [observers.Hdf5Logger(filename)])
        write_collada_animation(self.world,
                                join(tempdir, 'simplearm_animation.dae'),
                                filename)

    def test_scene(self):
        write_collada_scene(self.world, join(tempdir, 'simplearm_scene.dae'))

if __name__ == '__main__':
    unittest.main()

