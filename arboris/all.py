# coding=utf-8

"""Import all useful functions, classes and modules from arboris. 

This is meant for interactive use.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from arboris import *
from controllers import WeightController, ProportionalDerivativeController
from core import World, Body, Joint, JointsList,\
    NamedObjectsList, Frame, SubFrame, MovingSubFrame, simulate, Constraint,\
    Controller, Observer
from robots.human36 import add_human36
from robots.simpleshapes import add_sphere, add_box, add_cylinder,\
    add_groundplane
from robots.simplearm import add_simplearm
from robots.snake import add_snake

from numpy import arange
