# coding=utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from arboristest import TestCase
from arboris.all import *
import unittest
import tempfile
#from arboris.core import World, simulate, SubFrame
#from arboris.homogeneousmatrix import transl
#from numpy import array, arange, eye
from os.path import join
tempdir = tempfile.gettempdir()

class VisuColladaTestCase(TestCase):

    def setUp(self):
        self.world = World()
        self.world.register(WeightController())
        add_simplearm(self.world, with_shapes=True)
        self.world.getjoints()['Shoulder'].gpos[0] = 3.14/4
        self.cases = {True: 'flat', False:'notflat'}

    def test_animation(self):
        h5_filenames = {}
        dae_filenames = {}
        for flat, name in self.cases.iteritems():
            h5_filenames[flat] = join(tempdir,
                                      'simplearm_simulation_' + name +'.h5')
            dae_filenames[flat] = join(tempdir,
                                       'simplearm_animation_' + name +'.dae')
        obs = []
        #try:
        #    from arboris.visu_osg import OsgObserver
        #    obs = [OsgObserver()]
        #except ImportError:
        #    pass
        for flat in self.cases.iterkeys():
            obs.append(observers.Hdf5Logger(h5_filenames[flat],
                                            mode='w',
                                            flat=flat))
        simulate(self.world, arange(0, .1, .01), obs)
        for flat in self.cases.iterkeys():
            write_collada_animation(self.world,
                                    dae_filenames[flat],
                                    h5_filenames[flat],
                                    flat=flat)

    def test_scene(self):
        self.world.update_geometric()
        for flat, name in self.cases.iteritems():
            dae_filename = join(tempdir, 'simplearm_scene_' + name +'.dae')
            write_collada_scene(self.world, dae_filename, flat=flat)

if __name__ == '__main__':
    unittest.main()

