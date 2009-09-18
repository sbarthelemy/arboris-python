# coding=utf-8
"""
This example shows how to use BalanceController.

"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")

from arboris.core import ObservableWorld, simulate, SubFrame
from arboris.homogeneousmatrix import transl
from arboris.robots import simplearm
from arboris.visu_osg import Drawer
from arboris.qpcontroller import BalanceController
#from arboris.controllers import WeightController
from numpy import arange

world = ObservableWorld()
world.observers.append(Drawer(world))
simplearm.add_simplearm(world)
world.register(SubFrame(world.ground, transl(0.5,0.5,0), 'Target'))

# set initial position
joints = world.getjointslist()
joints[0].gpos = [0.1]

#world.register(WeightController(world))
world.register(BalanceController(world))

time = arange(0, 20., 0.005)
simulate(world, time)
