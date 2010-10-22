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

with `R^{-1}=R^T \in \Re^{3\times3}` and `p \in \Re^{3\times1}`.

The *pose* (position and orientation, also known as the *configuration*)
of a (right-handed) coordinate frame `\Psi_b` regarding to a reference 
(right-handed) coordinate frame `\Psi_a`: can be described by an 
homogeneous matrix

.. math::
    H_{ab} = 
    \begin{bmatrix}
        R_{ab} & p_{ab} \\
        \begin{smallmatrix}
            0 & 0 & 0
        \end{smallmatrix} & 1
    \end{bmatrix}

with:

- `p_{ab}` defined as the `3 \times 1` column vector of coordinates of 
  the origin of `\Psi_b` expressed in `\Psi_a`.

- `R_{ab}` defined as the `3 \times 3` matrix with the columns equal to
  the coordinates of the three unit vectors along the frame axes of 
  `\Psi_b` expressed in `\Psi_a`.


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

Wrenches
========

A generalized force acting on a rigid body consist in a linear component
(pure force) `f` and angular component (pure moment) `\tau`. The 
pair force/moment is named a *wrench* and can be represented using 
a vector in `R^6`:

.. math::
    \wrench[c] = 
    \begin{bmatrix}
        \pre[c]\tau(t)\\
        \pre[c]f(t)\\
    \end{bmatrix}

TODO: relative wrenches and power formulas.

Acceleration of a coordinate frame
==================================

TODO: introduce adjacency

Newton-Euler equations for a rigid body
=======================================

.. math::
    \begin{bmatrix}
        \pre[b]{\mathcal{I}} & 0   \\
        0                   & m I
    \end{bmatrix}
    \begin{bmatrix}
        \pre[b]{\dot{\omega}}_{b/g}(t) \\
        \pre[b]{\dot{v}}_{b/g}(t)
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 & \pre[b]\omega_{b/g}(t) \times \pre[b]{\mathcal{I}} \\
        0 & \pre[b]\omega_{b/g}(t) \times
    \end{bmatrix}
    \begin{bmatrix}
        \pre[b]\omega_{b/g}(t) \\
        \pre[b]v_{b/g}(t)
    \end{bmatrix}
    =
    \begin{bmatrix}
        \pre[b]\tau(t)\\
        \pre[b]f(t)\\
    \end{bmatrix}
    
where `\pre[b]{\mathcal{I}}` is the body inertial tensor, expressed 
in the body frame, `b`

Implementation
==============

The modules :mod:`twistvector`, :mod:`homogeneousmatrix` and 
:mod:`adjointmatrix` respectively  implement "low level" operations on 
twist and on homogeneous and adjoint matrices.
For instance, the following excerp creates the homogeneous matrix of a 
translation and then inverts it.

.. doctest::

  >>> import arboris.homogeneousmatrix as homogeneousmatrix
  >>> H = homogeneousmatrix.transl(1., 0., 2./3.)
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

A more convenient way of dealing with rigid motion is planned, by using
a child class of :class:`rigidmotion.RigidMotion`,  which wraps all the 
elementary functions in an object-oriented way. However, this child 
class does not exist yet, one may use :class:`rigidmotion.FreeJoint` 
(see next chapter) instead.


Dynamics
========

TODO: document 1st and 2nd order dynamics for a single rigid body.
