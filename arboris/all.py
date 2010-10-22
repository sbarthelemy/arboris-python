# coding=utf-8

"""Import all useful functions, classes and modules from arboris.

This is meant for interactive use.
"""
__author__ = (u"Sébastien BARTHÉLEMY <barthelemy@crans.org>")

#pylint: disable-msg=W0611
#pylint: disable-msg=W0614

from arboris import * #pylint: disable-msg=W0401
from arboris.controllers import WeightController, \
                                ProportionalDerivativeController
from arboris.core import World, Body, Joint, JointsList, NamedObjectsList, \
                         Frame, SubFrame, MovingSubFrame, simulate, \
                         Constraint, Controller, Observer
from arboris.robots.human36 import add_human36
from arboris.robots.simpleshapes import add_sphere, add_box, add_cylinder, \
                                        add_groundplane
from arboris.robots.simplearm import add_simplearm
from arboris.robots.snake import add_snake
from arboris.visu_collada import write_collada_animation, write_collada_scene, \
        view, ColladaDriver
from numpy import arange, dot, allclose, pi
