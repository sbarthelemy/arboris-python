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
joints which form a human model, plus the "ground" body. That world 
has 42 = 36 + 6 degrees of freedom. 

.. doctest::

  >>> from human36 import human36
  >>> (w, foo, foo2) = human36()
  >>> w.ndof

One can get a python dictionary of the bodies and joints:

.. doctest::

  >>> bodies = w.getbodiesdict()
  >>> joints = w.getjointsdict()
  >>> bodies['Head'].mass
  >>> joints['ElbowL'].gpos


Once the forward geometric model is computed, bodies poses can be queried

.. doctest::

  >>> w.update_geometric()
  >>> bodies['HandL'].pose
  array([[ 1.       ,  0.       ,  0.       ,  0.       ],
         [ 0.       ,  1.       ,  0.       ,  0.8943517],
         [ 0.       ,  0.       ,  1.       , -0.2254595],
         [ 0.       ,  0.       ,  0.       ,  1.       ]])
  >>> joints['ElbowL'].gpos[0] = 3.14/2
  >>> w.update_geometric()
  >>> bodies['HandL'].pose

Each body defines a frame, yet additionnal frames are often defined. They are 
called "subframes" in arboris terminology. A python dictionnary of all the
frames (body frames + subframes) can also be obtained.

Here, in addition to the left hand, we consider its third dactylion. The third
dactylion subframe is rigidly attached to the left hand, with the relative
pose given by the ``bpose`` property."

.. doctest::

  >>> frames =  w.getframesdict()
  >>> frames['HandL'].pose
  >>> frames['Left 3rd dactylion'].pose
  >>> frames['Left 3rd dactylion'].body.name
  'HandL'
  >>> frames['Left 3rd dactylion'].bpose
  array([[ 1.       ,  0.       ,  0.       ,  0.       ],
         [ 0.       ,  1.       ,  0.       , -0.1899431],
         [ 0.       ,  0.       ,  1.       ,  0.       ],
         [ 0.       ,  0.       ,  0.       ,  1.       ]])

In addition to the forward geometric model, the full kinematic and
dynamical models can be computed by the ``update_dynamic()``
method. Let's illustrate this on a smaller model, to avoid dealing with huge 
matrices here. We'll use a 3 links robot.

.. doctest::

  >>> from triplehinge import triplehinge
  >>> from numpy import dot, pi
  >>> w = triplehinge(lengths=(1., 1., 0.2))
  >>> joints = w.getjointsdict()
  >>> joints['Shoulder'].gpos[0] = pi/3
  >>> joints['Elbow'].gpos[0] = -2*pi/3
  >>> joints['Wrist'].gpos[0] = pi/3
  >>> joints['Wrist'].gvel[0] = pi/18
  >>> w.update_dynamic()
  >>> frames = w.getframesdict()
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


Drawing a robot
===============

TODO: explain how to use the visu

Inverse kinematics
==================

A dynamic simulation
====================

.. doctest::

  >>> w.mass
  >>> w.viscosity
  >>> w.nleffects
  >>> 


Using a controller
==================

Writing a controller
====================

Adding contacts
===============

