# coding=utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

import arboristest
from arboris.all import *
from os.path import join

class VisuColladaTestCase(arboristest.TestCase):

    def setUp(self):
        self.world = World()
        self.world.register(WeightController())
        add_simplearm(self.world, with_shapes=True)
        self.world.getjoints()['Shoulder'].gpos[0] = 3.14/4
        self.world.update_geometric()
        self.cases = {True: 'flat', False:'notflat'}
        self.h5_files = {}
        for flat, name in self.cases.iteritems():
            self.h5_files[flat] = join(self.testdir, 'simplearm_'+name+'.h5')
        if False:
            # save hdf5 files as a reference
            obs = []
            for flat in self.cases.iterkeys():
                obs.append(observers.Hdf5Logger(self.h5_files[flat],
                                                mode='w',
                                                flat=flat))
            simulate(self.world, arange(0, 1, .01), obs)

    def _test_anim(self, flat):
        name = self.cases[flat]
        anim_file = join(self.destdir, 'simplearm_anim_' + name +'.dae')
        scene_file = join(self.destdir, 'simplearm_scene_' + name +'.dae')
        write_collada_scene(self.world, scene_file, flat=flat)
        write_collada_animation(anim_file, scene_file, self.h5_files[flat])
        if self.interactive:
            view_collada_animation(scene_file, self.h5_files[flat])

    def test_anim_flat(self):
        self._test_anim(True)

    def test_anim_notflat(self):
        self._test_anim(False)

if __name__ == '__main__':
    arboristest.main()
