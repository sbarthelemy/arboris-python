# coding=utf-8
"""
This example shows how to use BalanceController.

"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")

from arboris.core import World, simulate, SubFrame
from arboris.homogeneousmatrix import transl
from arboris.robots.simplearm import add_simplearm
from arboris.visu_osg import Drawer
from arboris.qpcontroller import BalanceController, Task
#from arboris.controllers import WeightController
from numpy import arange

world = World()
add_simplearm(world, name='Left')
world.register(SubFrame(world.ground, transl(0.5, 0.5, 0), 'LeftTarget'))
joints = world.getjoints()
frames = world.getframes()
# set initial position:
joints[0].gpos = [0.1]

task = Task(controlled_frame=frames['LeftEndEffector'],
         target_frame=frames['LeftTarget'], 
         world=world)

#world.register(WeightController())
world.register(BalanceController(world, [task]))

from arboris.core import JointsList
add_simplearm(world, name='Right')
joints=world.getjoints()
J=JointsList(joints[0:3])

#time = arange(0, 20., 0.005)
#simulate(world, time)
