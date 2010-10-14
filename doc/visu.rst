=============
Visualization
=============

The visualisation of a simulation is still a work
in progress. However it is quite useable right now.

Viewing the simulation involves four steps:

1. generate a collada file describing the scene
2. run the simulation and save the trajectories in an HDF5 file
3. combine the scene and the trajectories into another collada file
   describing the animated scene.
4. view the animation using an external viewer such as `dænim
   <http://github.com/sbarthelemy/daenim>`_ or blender.

>>> def dest(filename):
...     """return absolute path to ``filename`` in an temporary directory.
...
...     The temporary directory is platform-dependant. It is usually
...     ``/tmp`` or ``$TMPDIR``.
...
...     """
...     import tempfile
...     import os
...     return os.path.join(tempfile.gettempdir(), filename)
>>> from arboris.all import *
>>> world = World()
>>> world.register(WeightController())
>>> add_simplearm(world, with_shapes=True)
>>> world.getjoints()['Shoulder'].gpos[0] = 3.14/4
>>> write_collada_scene(world, dest('myscene.dae')) # step 1
>>> obs = [observers.Hdf5Logger(dest('mysimu.h5'))]
>>> simulate(world, arange(0, 1, .01), obs) # step 2
>>> write_collada_animation(dest('myanim.dae'), dest('myscene.dae'),
...                         dest('mysimu.h5')) # step 3
>>> import subprocess
>>> view(dest('myanim.dae')) # step 4 #doctest:+SKIP

The :func:`write_collada_animation` function is a simple wrapper around the
external ``h5toanim`` command from the `ColladaTools project
<http://github.com/sbarthelemy/ColladaTools>`_.

The last two steps can be combined using the convenience function
:func:`view`. Using it, the last three commands from the
above example would be replaced by this single one:

>>> view(dest('myscene.dae'), dest('mysimu.h5')) #doctest:+SKIP

Customizing the visualization
=============================

One can skip the step 1 and provide its own collada scene file to animate.
One can also edit the generated the generated scene either manually or with
a filter.

It is also possible to pass some basic options to the scene generator, to
change the color of bodies, the size of the frame arrows or to toggle the
display of shapes.

Passing options to the scene generator
--------------------------------------

To use these options, one cannot use the :func:`write_collada_scene` helper
function.

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
>>> driver = ColladaDriver(dest('mycustomizedscene.dae'), options=options)

.. autoclass:: arboris._visu.ColorGenerator
   :members:
   :undoc-members:

In the following example, we set the colors of the arm and the hand to be
respectively blue and green.
The forearm color is generated automatically.
The shapes inherit the color of the body they are attached to, with the
exception of the end-effector point which color was set to gray (otherwise,
being attached to the hand, it would have been green).

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
>>> view(dest('mycustomizedscene.dae')) #doctest:+SKIP

Automatically editing the generated scene
-----------------------------------------

Rescale a plane
~~~~~~~~~~~~~~~

Here is a trivial scene with two planes. The collisison detection considers the
planes are infinite, but they are drawn as (finite) squares. The size of
the squares can be controlled through the 'plane half extents' option.
However this option will change the size of all the planes. To alter
a single plane, one can use :mod:`ElementTree` to edit the generated scene
as shown here.

>>> world = World()
>>> world.register(shapes.Plane(world.ground, coeffs=(0, 1, 0, 0), name="LowerPlane"))
>>> f = SubFrame(world.ground, homogeneousmatrix.transl(0, 1, 0))
>>> world.register(shapes.Plane(f, coeffs=(0, 1, 0, 0), name="UpperPlane"))
>>> write_collada_scene(world, dest('twoplanes.dae'))


>>> def rescale_upper_plane(tree, factor):
...     from arboris.visu_collada import find_by_id, QN
...     node_el = find_by_id(tree, 'UpperPlane', QN('node'))
...     scale_el = node_el.find(QN('scale'))
...     scales = []
...     for t in scale_el.text.split():
...         scales.append(float(t) * factor)
...     scale_el.text = "{0} {1} {2}".format(*scales)
>>> import xml.etree.ElementTree as ET
>>> tree = ET.parse(dest('twoplanes.dae'))
>>> rescale_upper_plane(tree, .5)
>>> visu_collada.indent(tree)
>>> visu_collada.fix_namespace(tree)
>>> with open(dest('twoplanes_rescaled.dae'), 'w') as f:
...     tree.write(f, "utf-8")
>>> view(dest('twoplanes_rescaled.dae')) #doctest:+SKIP


Add meshes
~~~~~~~~~~

TODO

Under the hood
==============

The details of how to describe something with collada are hidden in the
:class:`arboris._visu.DrawerDriver` interface. It is used by the
:class:`arboris._visu.Drawer` helper class to create the scene representing
a world.
