=============
Visualization
=============

The visualisation of a simulation is still a work
in progress. However it is quite useable right now.

There are two ways of viewing the simulation:
online, during the simulation (using osgswig),
and offline, playing a generated collada animation
file.


Online
======

Viewing the simulation online is a simple matter of using
the proper observer.

>>> from arboris.all import *
>>> world = World()
>>> world.register(WeightController())
>>> add_simplearm(world)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4
>>> simulate(world, arange(0, 1, .01), [OsgObserver()])

Offline
=======

Viewing the simulation offline is done by saving the
simulation data into an hdf5 file and then generating the
animation file.

This file can later be played in blender.

>>> from arboris.all import *
>>> world = World()
>>> world.register(WeightController())
>>> add_simplearm(world)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4
>>> simulate(world, arange(0, 1, .01), [Hdf5Logger('mysimu.h5')])
>>> write_collada_animation(world, 'mysimu.dae', 'mysimu.h5')

Under the hood
==============

The details of how to draw something in osg or how to describe it
with collada are hidden behind the :class:`arboris._visu.DrawerDriver`
and :class:`arboris._visu.AnimatorDriver` interfaces. The former is
used by the :class:`arboris._visu.Drawer` helper class to create the
scene repesenting a world.

