================
Rigid Mechanisms
================

We define a rigid mechanism as a finite number of rigid bodies interconnected by ideal joints.

Ideal Joints
============

An ideal joint is a kinematic restriction of the allowed relative twist of two rigid bodies `i` and `j` to a linear subspace of dimension `k`, where the relative motion of the bodies is described by two sets of states, namely 

- a matrix `\GPos`, parameterizing the relative configuration as `\Hg[i]_j = \Hg[i]_j(\GPos)`,
- a vector `\GVel \in \Re^k`, parameterizing the relative twist as `\twist[i]_{i/j} = X(\GPos) \GVel`

where `X(\GPos)` depends smoothly on `\GPos` and `\nu = V_\GPos(\dot{\GPos})` with `V_\GPos` invertible and linear in `\dot{\GPos}`. Furthermore, there exists a mapping `F_\GPos : \Re^k \rightarrow \GPosSet`.

Implementation
==============

Several ideal joints are already implemented:

- :class:`RzJoint` for hinge joints,
- :class:`RzRyJoint`, :class:`RyRxJoint` and :class:`RzRxJoint` for cardan 
  joints, 
- :class:`RzRyRxJoint` for ball and socket joints,
- :class:`FreeJoint` for "free" joints, that do not constrain the relative
  motion.

Let's take the example of an hinge joint, it has 1 dof, and may be parametrized by the angle `q` and its derivative `\dot{q}`. Here we create a joint, with angle of 60 degrees and rotational velocity of 2 radians per second.

.. doctest::

  >>> from arboris import *
  >>> j = RzJoint(gpos = 3.14/3, gvel = 2.)
  >>> j.ndof()
  1
  >>> j.gpos
  >>> j.gvel

The relative configuration corresponding to this joint is defined by the homogeneous `\Hg[r]_n` is given by ``pose()``

.. math::

  \Hg[r]_n =
  \begin{bmatrix}
  cos(q) & -sin(q) & 0 & 0\\
  sin(q) &  cos(q) & 0 & 0\\
  0      &  0      & 1 & 0\\
  0      &  0      & 0 & 1       
  \end{bmatrix}

.. doctest::

  >>> j.pose()
  array([[ 0.50045969, -0.86575984,  0.        ,  0.        ],
         [ 0.86575984,  0.50045969,  0.        ,  0.        ],
         [ 0.        ,  0.        ,  1.        ,  0.        ],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])

Its inverse, `\Hg[n]_r = \Hg[r]_n^{-1}` is given by ``ipose()``

.. doctest::

  >>> j.ipose()
  array([[ 0.50045969,  0.86575984,  0.        ,  0.        ],
         [-0.86575984,  0.50045969,  0.        ,  0.        ],
         [ 0.        ,  0.        ,  1.        ,  0.        ],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])

Similarly, the relative twist `\twist[n]_{n/r}` is given by ``twist()`` and its inverse by ``itwist()``

.. doctest::

  >>> j.twist()
  array([ 0.,  0.,  2.,  0.,  0.,  0.])
  >>> j.itwist()
  array([ 0.,  0., -2.,  0.,  0.,  0.])

Eventually, the `X(\GPos)` matrix, which we (perhaps improperly) call 
jacobian, is given by ``jacobian()``

.. doctest::

  >>> j.jacobian()
  array([ 0.,  0.,  1.,  0.,  0.,  0.])

Mechanisms
==========

Joints an bodies are interconnected in a tree-like structure, whose nodes are the bodies and edges are the joints. More precisely, a joint is connected between to *frames*, each belonging to a different body.



Bodies and frames
=================



