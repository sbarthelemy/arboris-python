# coding=utf-8
"""
This example shows that arboris creates energy.

We can see it many simulations, by toggling:
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from arboristest import TestCase
import h5py
from arboris.all import *
import unittest

def run_simulation(is_fixed=False, with_weight=True):
    world = World()
    njoints = 9
    lengths = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
    masses = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
    gpos = [0, 3.14159/4., 0, 0, 0, 0, 0, 0, 0]
    gvel = gvel=[2.]*njoints
    add_snake(world, njoints, lengths=lengths, masses=masses, gpos=gpos,
              gvel=gvel, is_fixed=False)
    if with_weight:
        world.register(controllers.WeightController())
        t_end = 2.08
    else:
        t_end = 1.430
    nrj = observers.EnergyMonitor()      
    timeline = arange(0, t_end, 0.005)
    simulate(world, timeline, (nrj,))

    if False:
        # save result to a file
        g = h5py.File('tests/energy_drift.h5', 'a')
        if is_fixed:
            name = 'fixed_'
        else:
            name = 'free_'
        if with_weight:
            name += 'with_weight'
        else:
            name += 'without_weight' 
        g[name] = nrj.kinetic_energy
        g.close()
    #nrj.plot()
    return nrj

class EnergyDriftTestCase(TestCase):

    def test_free_with_weight(self):
        nrj = run_simulation(False, True)
        g = h5py.File('tests/energy_drift.h5', 'r')
        try:
            self.assertListsAlmostEqual(g['free_with_weight'],
                                        nrj.kinetic_energy)
        finally:
            g.close()
 
if __name__ == '__main__':
    unittest.main()


    

