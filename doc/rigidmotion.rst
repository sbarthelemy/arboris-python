============
Rigid Motion
============

Rigid bodies and frames
=======================

TODO: Define *body* and *body twist*

Position of a coordinate frame
==============================

An homogeneous matrix `H` is a matrix of the form

.. math::
    H = 
    \begin{bmatrix}
        R & p \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}
    \in \Re^{4\times4}

with :math:`R^{-1}=R^T \in \Re^{3\times3}` and :math:`p \in \Re^{3\times1}`.

The *pose* (position and orientation, also known as the *configuration*) of a (right-handed) coordinate frame :math:`\Psi_b` regarding to a reference (right-handed) coordinate frame :math:`\Psi_a`: can be described by an homogeneous matrix

.. math::
    \Hg[a]{b} = 
    \begin{bmatrix}
        \pre[a]R_b & \pre[a]p_b \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}

with:

- :math:`\pre[a]p_b` defined as the :math:`3 \times 1` column vector of coordinates of the origin of :math:`Psi_b` expressed in :math:`\Psi_a`.

- :math:`\pre[a]R_b` defined as the :math:`3 \times 3` matrix with the columns equal to the coordinates of the three unit vectors along the frame axes of :math:`\Psi_b` expressed in :math:`\Psi_a`.


Velocity of a coordinate frame
==============================

The velocity of a rigid body can be described by a twist.

.. math::
    \twist[c]_{a/b} = 
    \begin{bmatrix}
        \pre[c]\omega_{a/b}(t)\\
        \pre[c]v_{a/b}(t)\\
    \end{bmatrix}

TODO: add adjoint matrix and relative velocities formulas

Acceleration of a coordinate frame
==================================

TODO: introduce adjacency

Implementation
==============

The modules :mod:`twistvector`, :mod:`homogeneousmatrix` and :mod:`adjointmatrix` respectively  implement "low level" operations on twist and on homogeneous and adjoint matrices. For instance, 
the following excerp creates the homogeneous matrix of a translation and then inverts it.

.. doctest::

  >>> import homogeneousmatrix
  >>> H = homogeneousmatrix.transl((1., 0., 2./3.))
  >>> H
  array([[ 1.        ,  0.        ,  0.        ,  1.        ],
         [ 0.        ,  1.        ,  0.        ,  0.        ],
         [ 0.        ,  0.        ,  1.        ,  0.66666667],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])
  >>> Hinv = homogeneousmatrix.inv(H)
  >>> Hinv
  array([[ 1.        ,  0.        ,  0.        , -1.        ],
         [ 0.        ,  1.        ,  0.        , -0.        ],
         [ 0.        ,  0.        ,  1.        , -0.66666667],
         [ 0.        ,  0.        ,  0.        ,  1.        ]])


A more convenient way of dealing with rigid motion is planned, by using a child class of :class:`rigidmotion.RigidMotion`,  which wraps all the elementary functions in an object-oriented way. However, this child class does not exist yet, one may use :class:`rigidmotion.FreeJoint` (see next chapter) instead.


Dynamics
========

TODO: document 1st and 2nd order dynamics for a single rigid body.
