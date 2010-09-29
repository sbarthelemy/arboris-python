=============
Visualization
=============

The visualisation of a simulation is still a work
in progress. However it is quite useable right now.

Viewing the simulation involves four steps:

1. generate a collada file describing the initial scene
2. run the simulation and save the trajectories in an HDF5 file
3. combine the scene and the trajectories into another collada file
   describing the animated scene.
4. view the animation using an external viewer such as `dænim
   <http://github.com/sbarthelemy/daenim>`_ or blender.


>>> from arboris.all import *
>>> world = World()
>>> world.register(WeightController())
>>> add_simplearm(world)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4
>>> write_collada_scene(world, 'myscene.dae') # step 1
>>> simulate(world, arange(0, 1, .01), [Hdf5Logger('mysimu.h5')]) # step 2
>>> write_collada_animation('myanim.dae', 'myscene.dae', 'mysimu.h5') # step 3
>>> import subprocess
>>> subprocess.call(['daenim', 'myanim.dae']) # step 4

The :func:`write_collada_animation` function is a simple wrapper around the
external ``h5toanim`` command from the `ColladaTools project
<http://github.com/sbarthelemy/ColladaTools>`_.

The last two steps can be combined using the convenience function
:func:`view_collada_animation`. Using it, the last three lines from the above
example would be replaced by this single line:

>>> view_collada_animation('myscene.dae', 'mysimu.h5')

Obviously, one can also skip step 1 and provide its own collada scene file or
manually customize the generated one.

Under the hood
==============

The details of how to describe something with collada are hidden in the
:class:`arboris._visu.DrawerDriver` interface. It is used by the
:class:`arboris._visu.Drawer` helper class to create the scene representing
a world.
