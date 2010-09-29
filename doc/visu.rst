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
>>> add_simplearm(world, with_shapes=True)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4
>>> write_collada_scene(world, 'myscene.dae') # step 1
>>> simulate(world, arange(0, 1, .01), [observers.Hdf5Logger('mysimu.h5')]) # step 2
>>> write_collada_animation('myanim.dae', 'myscene.dae', 'mysimu.h5') # step 3
>>> import subprocess
>>> subprocess.check_call(['daenim', 'myanim.dae']) # step 4
0

The :func:`write_collada_animation` function is a simple wrapper around the
external ``h5toanim`` command from the `ColladaTools project
<http://github.com/sbarthelemy/ColladaTools>`_.

The last two steps can be combined using the convenience function
:func:`view_collada_animation`. Using it, the last three lines from the above
example would be replaced by this single line:

>>> view_collada_animation('myscene.dae', 'mysimu.h5')

Customizing the visualization
============================

One can skip the step 1 and provide its own collada scene file. One can
also edit the generated the generated scene either manually or with a filter.

It is also possible to pass some basic options to the scene generator, to
change the color of bodies, the size of the frame arrows or to toggle the
display of shapes.

To use these options, one cannot use the :func:`write_collada_scene` helper
function.

>>> from arboris.all import *
>>> import subprocess
>>> world = World()
>>> world.register(WeightController())
>>> add_simplearm(world, with_shapes=True)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4

Some graphic elements have a fixed size, such as the points or the frame
arrows. They can be scaled altogether with a single :param:`scale` parameter,
and ca

>>> options = ColladaDriver.get_default_options(scale=1.)
>>> print(options['point radius'])
0.008
>>> print(options['frame arrows length'])
0.08
>>> options = ColladaDriver.get_default_options(scale=1.5)
>>> print(options['point radius'])
0.012
>>> print(options['frame arrows length'])
0.12
>>> options['point radius'] = .025
>>> driver = ColladaDriver('mycustomizedscene.dae', options=options)

.. autoclass:: arboris._visu.ColorGenerator
   :members:
   :undoc-members:

In the following example, we set the colors of the arm and the hand to be
respectively blue and green.
The forearm color is generated automatically.
The shapes inherit the color of the body they are attached to, with the
exception of the end-effector point which color was set to gray (otherwise,
being attached to the hand, it would have been green)

>>> from arboris._visu import ColorGenerator, Drawer
>>> hand = world.getbodies()['Hand']
>>> point = shapes.Point(world.getframes()['EndEffector'])
>>> world.register(point)
>>> cg = ColorGenerator({world.getbodies()['Arm']: (0, 0, 1),
...                      hand: (0, 1, 0),
...                      point: (1, 1, 0)})
>>> drawer = Drawer(driver, color_generator=cg)
>>> world.parse(drawer)
>>> drawer.finish()
>>> subprocess.check_call(['daenim', 'mycustomizedscene.dae'])
0

Under the hood
==============

The details of how to describe something with collada are hidden in the
:class:`arboris._visu.DrawerDriver` interface. It is used by the
:class:`arboris._visu.Drawer` helper class to create the scene representing
a world.
