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

    def test_animation(self):
        h5_filenames = {}
        anim_filenames = {}
        scene_filenames = {}
        for flat, name in self.cases.iteritems():
            h5_filenames[flat] = join(self.destdir,
                                      'simplearm_simulation_' + name +'.h5')
            anim_filenames[flat] = join(self.destdir,
                                        'simplearm_animation_' + name +'.dae')
            scene_filenames[flat] = join(self.destdir,
                                         'simplearm_scene_' + name +'.dae')
        obs = []
        for flat in self.cases.iterkeys():
            obs.append(observers.Hdf5Logger(h5_filenames[flat],
                                            mode='w',
                                            flat=flat))
        simulate(self.world, arange(0, .1, .01), obs)
        for flat in self.cases.iterkeys():
            write_collada_scene(self.world, scene_filenames[flat], flat=flat)
            write_collada_animation(anim_filenames[flat],
                    scene_filenames[flat], h5_filenames[flat])
            if self.interactive:
                view_collada_animation(scene_filenames[flat], h5_filenames[flat])

if __name__ == '__main__':
    arboristest.main()
