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

  >>> from robots import simplearm
  >>> w = simplearm()
  >>> w.ndof
  3

One can get a python dictionary of the bodies and joints:

.. doctest::

  >>> bodies = w.getbodiesdict()
  >>> joints = w.getjointsdict()
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

  >>> frames =  w.getframesdict()
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

  >>> from robots import simplearm
  >>> from numpy import dot, pi
  >>> w = simplearm(lengths=(1., 1., 0.2))
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

The simplest way to have a graphic representation of a world is to use :class:`DrawableWorld` objects instead of :class:`World` ones. The :class:`DrawableWorld` class inherits form the :class:`World` one, thus everythng we've learnt before  still holds. It adds 3 methods :method:`init_graphic:`, :method:`update_graphic` and :method:`graphic_is_done`.



.. doctest::

  >>> from robots import simplearm
  >>> w = simplearm()
  >>> joints = w.getjointsdict()
  >>> t = 0
  >>> dt = 1/10
  >>> while(not(w.graphic_is_done())):
  >>>     t += dt
  >>>     joints['Shoulder'].gpos[0] = t
  >>>     joints['Elbow'].gpos[0] = -2*t
  >>>     joints['Wrist'].gpos[0] = t
  >>>     w.update_geometric()
  >>>     w.update_graphic()


Inverse kinematics
==================

A dynamic simulation
====================

.. doctest::

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
  >>> 


Using a controller
==================

Writing a controller
====================

Adding contacts
===============

