

from arboris.core import World, simulate, SubFrame
from arboris.homogeneousmatrix import transl
from numpy import array, arange, eye
from arboris.controllers import WeightController
from arboris.observers import Hdf5Logger

time = arange(0, .05, 0.01)
file_name = "the_temporary_file_where_we_save_simulation_data.h5"
dest_in_file = "test/xp"

world = World()
world.register( WeightController() )
h5obs = Hdf5Logger(world, len(time)-1, file_name, dest_in_file, 'w')

from arboris.robots import simplearm
simplearm.add_simplearm(world)
world.getjoints()[0].gpos = array([0.1])

simulate(world, time,[h5obs] )

import os
os.remove(file_name)
