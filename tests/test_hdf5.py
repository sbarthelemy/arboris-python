

from arboris.core import ObservableWorld, simulate, SubFrame, WorldObserver
from arboris.homogeneousmatrix import transl
from numpy import array, arange, eye
from arboris.controllers import WeightController
from arboris.observers import Hdf5Logger

time = arange(0, .05, 0.01)
file_name = "the_temporary_file_where_we_save_simulation_data.h5"
dest_in_file = "test/xp"

world = ObservableWorld()
world.register( WeightController(world) )
h5obs = Hdf5Logger(world, len(time)-1, file_name, dest_in_file, 'w')
world.observers.append(h5obs)

from arboris.robots import simplearm
simplearm.add_simplearm(world)
world.getjoints()[0].gpos = array([0.1])

simulate(world, time)

import os
os.remove(file_name)
