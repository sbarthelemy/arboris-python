================
Rigid Mechanisms
================

We define a rigid mechanism as a finite number of rigid bodies interconnected by ideal joints.

Ideal Joints
============

An ideal joint is a kinematic restriction of the allowed relative twist of two rigid bodies :math:`i` and :math:`j` to a linear subspace of dimension :math:`k`, where the relative motion of the bodies is described by two sets of states, namely 

- a matrix :math:`\GPos`, parameterizing the relative configuration as :math:`\Hg[i]_j = \Hg[i]_j(\GPos)`,
- a vector :math:`\GVel \in \Re^k`, parameterizing the relative twist as :math:`\twist[i]_{i/j} = X(\GPos) \GVel`

where :math:`X(\GPos)` depends smoothly on :math:`\GPos` and :math:`\nu = V_\GPos(\dot{\GPos})` with :math:`V_\GPos` invertible and linear in :math:`\dot{\GPos}`. Furthermore, there exists a mapping :math:`F_\GPos : \Re^k \rightarrow \GPosSet`.

Implementation
==============

Currently, only 3 ideal joints are implemented :
- :class:`HingeJoint`
- :class:`FreeJoint`
- :class:`RzRyRzJoint`

Let's take the example of an hinge joint, it has 1 dof, and may be parametrized by the angle :math:`q` and its derivative :math:`\dot{q}`. Here we create a joint, with angle of approximatly 60 degrees and rotational velocity of 2 radians per second.

.. doctest::

  >>> from arboris import *
  >>> j = HingeJoint(gpos = 3.14/3, gvel = 2.)
  >>> j.ndof()
  1

The relative configuration corresponding to this joint is defined by the homogeneous :math:`\Hg[r]_n` is given by ``pose()``

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

For this joint, we recognize

Its inverse, :math:`\Hg[n]_r = \Hg[r]_n^{-1}` is given by ``ipose()``

.. doctest::

  >>> j.ipose()
  array([[ 0.50045969,  0.86575984,  0.        ,  0.        ],
         [-0.86575984,  0.50045969,  0.        ,  0.        ],
         [ 0.        ,  0.        ,  1.        ,  0.        ],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])

Similarly, the relative twist :math:`\twist[n]_{n/r}` is given by ``twist()`` and its inverse by ``itwist()``

.. doctest::

  >>> j.twist()
  array([ 0.,  0.,  2.,  0.,  0.,  0.])
  >>> j.itwist()
  array([ 0.,  0., -2.,  0.,  0.,  0.])



