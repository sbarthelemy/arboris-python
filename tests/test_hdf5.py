

from arboris.core import ObservableWorld, simulate, SubFrame, WorldObserver
from arboris.homogeneousmatrix import transl
from numpy import array, arange, eye
from arboris.controllers import WeightController
from arboris.observers import Hdf5Logger

world = ObservableWorld()
world.register( WeightController(world) )
tobs = Hdf5Logger(world)
world.observers.append(tobs)

from arboris.robots import simplearm
simplearm.add_simplearm(world)

world.getjoints()[0].gpos = array([0.1])

time = arange(0, .03, 0.01)
simulate(world, time)

xp_file_name = "the_temporary_file_where_we_save_simulation_data.h5"
tobs.write_file(xp_file_name, "test/xp", 'w')
import os
os.remove(xp_file_name)
