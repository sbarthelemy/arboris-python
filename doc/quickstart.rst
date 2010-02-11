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

One can get a list of the bodies[1]_:

.. doctest::

  >>> bodies = w.getbodies()
  >>> joints = w.getjoints()
  >>> bodies['ForeArm'].mass
  array([[  4.27733333e-02,   0.00000000e+00,   0.00000000e+00,
            0.00000000e+00,   0.00000000e+00,   1.60000000e-01],
         [  0.00000000e+00,   2.13333333e-04,   0.00000000e+00,
            0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   4.27733333e-02,
           -1.60000000e-01,   0.00000000e+00,   0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,  -1.60000000e-01,
            8.00000000e-01,   0.00000000e+00,   0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
            0.00000000e+00,   8.00000000e-01,   0.00000000e+00],
         [  1.60000000e-01,   0.00000000e+00,   0.00000000e+00,
            0.00000000e+00,   0.00000000e+00,   8.00000000e-01]])
  >>> joints['Elbow'].gpos
  array([ 0.])

.. [1] this is not a raw python list, but an instance of 
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
  >>> joints['Elbow'].gpos[0] = 3.14/2
  >>> w.update_geometric()
  >>> bodies['Hand'].pose
  array([[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
           -3.99999873e-01],
         [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
            5.00318531e-01],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
            0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
            1.00000000e+00]])


Each body defines a frame, yet additionnal frames are often defined. They are 
called "subframes" in arboris terminology. A python dictionnary of all the
frames (body frames + subframes) can also be obtained.

Here, in addition to the hand, we consider its extremity. The third
dactylion subframe is rigidly attached to the left hand, with the relative
pose given by the ``bpose`` property."

.. doctest::

  >>> frames =  w.getframes()
  >>> frames['Hand'].pose
  array([[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
           -3.99999873e-01],
         [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
            5.00318531e-01],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
            0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
            1.00000000e+00]])
  >>> frames['EndEffector'].pose
  array([[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
           -5.99999810e-01],
         [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
            5.00477796e-01],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
            0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
            1.00000000e+00]])
  >>> frames['EndEffector'].body.name
  'Hand'
  >>> frames['EndEffector'].bpose
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
  >>> frames['EndEffector'].pose
  array([[  1.00000000e+00,  -1.48806748e-17,   0.00000000e+00,
           -2.97613496e-18],
         [ -1.11022302e-16,   1.00000000e+00,   0.00000000e+00,
            1.20000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
            0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
            1.00000000e+00]])
  >>> frames['EndEffector'].twist
  array([ 0.        ,  0.        ,  0.17453293, -0.03490659,  0.        ,  0.        ])
  >>> frames['EndEffector'].jacobian
  array([[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
         [  1.00000000e+00,   1.00000000e+00,   1.00000000e+00],
         [ -1.20000000e+00,  -7.00000000e-01,  -2.00000000e-01],
         [  1.48806748e-17,   8.66025404e-01,   0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
  >>> dot(frames['EndEffector'].jacobian, w.gvel)
  array([ 0.        ,  0.        ,  0.17453293, -0.03490659,  0.        ,  0.        ])
  >>> w.mass
  array([[ 1.24417333,  0.20000667,  0.02267333],
         [ 0.20000667,  0.49000667,  0.01267333],
         [ 0.02267333,  0.01267333,  0.00267333]])
  >>> w.viscosity
  array([[ 0.,  0.,  0.],
         [ 0.,  0.,  0.],
         [ 0.,  0.,  0.]])
  >>> w.nleffects
  array([[ -1.57361875e-18,  -3.52139031e-18,  -5.19433541e-20],
         [ -3.02299894e-03,  -3.02299894e-03,  -3.02299894e-03],
         [ -1.38334127e-19,   0.00000000e+00,   0.00000000e+00]])


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

One of these observers gives a graphic representation of the world:
the class :class:`arboris.visu_osg.Drawer`.

.. doctest::

  >>> from arboris.all import *
  >>> from numpy import arange
  >>> from arboris.visu_osg import Drawer
  >>> w = World()
  >>> add_simplearm(w)
  >>> timeline = arange(0.,.1,1e-3)
  >>> simulate(w, timeline, observers=[Drawer(w)])

Using a controller
==================

.. doctest::

  >>> from arboris.all import *
  >>> from numpy import arange, diag, sqrt
  >>> from arboris.visu_osg import Drawer
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
  >>> simulate(w, timeline, [Drawer(w)])

Writing a controller
====================

TODO

Adding contacts
===============

TODO
