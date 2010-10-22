.. testsetup::

  >>> import numpy
  >>> numpy.set_printoptions(suppress=True)

===========
Quick start
===========

A world consists of 

- rigid bodies interconnected by ideal joints in a tree topology,
- controllers generating generalized forces at joint level,
- additional constraints such as contacts or kinematic joints.


First contact with the :class:`World`
=====================================

First, let's create a world ``w``, which will consist of bodies and 
joints which form a simple planar arm model, plus the "ground" body. 
That world has 3 degrees of freedom. 

.. doctest::

  >>> from arboris.all import *
  >>> w = World()
  >>> add_simplearm(w)
  >>> w.ndof
  3

One can get a list of the bodies [#]_ :

.. doctest::

  >>> bodies = w.getbodies()
  >>> joints = w.getjoints()
  >>> bodies['Forearm'].mass.shape
  (6, 6)
  >>> joints['Elbow'].gpos
  array([ 0.])

.. [#] this is not a raw python list, but an instance of
  :class:`arboris.core.NamedObjectsList` which allows to index elements
  by the object name in addition to integers.

Once the forward geometric model is computed, bodies poses can be queried

.. doctest::

  >>> w.update_geometric()
  >>> bodies['Hand'].pose
  array([[ 1. ,  0. ,  0. ,  0. ],
         [ 0. ,  1. ,  0. ,  0.9],
         [ 0. ,  0. ,  1. ,  0. ],
         [ 0. ,  0. ,  0. ,  1. ]])
  >>> joints['Elbow'].gpos[0] = pi/2
  >>> w.update_geometric()
  >>> bodies['Hand'].pose
  array([[ 0. , -1. ,  0. , -0.4],
         [ 1. ,  0. ,  0. ,  0.5],
         [ 0. ,  0. ,  1. ,  0. ],
         [ 0. ,  0. ,  0. ,  1. ]])

Each body defines a frame, yet additionnal frames are often defined. They are 
called "subframes" in arboris. A python dictionnary of all the
frames (body frames + subframes) can also be obtained.

Here, in addition to the hand, we consider its extremity. The third
dactylion subframe is rigidly attached to the left hand, with the relative
pose given by the ``bpose`` property."

.. doctest::

  >>> frames =  w.getframes()
  >>> frames['EndEffector'].pose # pose relative to the ground
  array([[ 0. , -1. ,  0. , -0.6],
         [ 1. ,  0. ,  0. ,  0.5],
         [ 0. ,  0. ,  1. ,  0. ],
         [ 0. ,  0. ,  0. ,  1. ]])
  >>> frames['EndEffector'].body.name # parent body
  'Hand'
  >>> frames['EndEffector'].bpose # pose relative to the parent body (constant)
  array([[ 1. ,  0. ,  0. ,  0. ],
         [ 0. ,  1. ,  0. ,  0.2],
         [ 0. ,  0. ,  1. ,  0. ],
         [ 0. ,  0. ,  0. ,  1. ]])


In addition to the forward geometric model, the full kinematic and
dynamical models can be computed by the ``update_dynamic()``
method.

.. doctest::

  >>> from arboris.all import *
  >>> from numpy import dot, pi
  >>> w = World()
  >>> add_simplearm(w,lengths=(1., 1., 0.2))
  >>> joints = w.getjoints()
  >>> joints['Shoulder'].gpos[0] = pi/3
  >>> joints['Elbow'].gpos[0] = -2*pi/3
  >>> joints['Wrist'].gpos[0] = pi/3
  >>> joints['Wrist'].gvel[0] = pi/18
  >>> w.update_dynamic()
  >>> frames = w.getframes()
  >>> twist = frames['EndEffector'].twist
  >>> twist.shape
  (6,)
  >>> jacobian = frames['EndEffector'].jacobian
  >>> jacobian.shape
  (6, 3)
  >>> allclose(twist, dot(jacobian, w.gvel))
  True
  >>> w.mass.shape
  (3, 3)
  >>> w.viscosity
  array([[ 0.,  0.,  0.],
         [ 0.,  0.,  0.],
         [ 0.,  0.,  0.]])
  >>> w.nleffects.shape
  (3, 3)

A dynamic simulation
====================

.. doctest::

  >>> from arboris.all import *
  >>> w = World()
  >>> add_simplearm(w,lengths=(1., 1., 0.2))
  >>> timeline = arange(0.,.1,1e-3)
  >>> simulate(w, timeline)


...with visualization
=====================

To get informations about the world, we can add
:class:`arboris.core.Observers` to the simulation.
One of these observers saves the trajectory of the frames for later viewing.
See :doc:`visu` for more details.

Using a controller
==================

.. doctest::

  >>> from arboris.all import *
  >>> from numpy import arange, diag, sqrt
  >>> from arboris.controllers import ProportionalDerivativeController
  >>> w = World()
  >>> add_simplearm(w)
  >>> joints = w.getjoints()
  >>> w.register(ProportionalDerivativeController(
  ...     joints,
  ...     gpos_des=(3.14/4,3.14/4,3.14/4),
  ...     kp=diag((1.,1.,1.)),
  ...     kd=diag((1.,1.,1.))/sqrt(2)))
  >>> timeline = arange(0.,3,1e-3)
  >>> simulate(w, timeline)

Writing a controller
====================

TODO

Adding contacts
===============

TODO
