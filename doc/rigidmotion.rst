============
Rigid Motion
============

Position
========

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


Velocity
========

The velocity of a rigid body can be described by a twist.

.. math::
    \twist[c]_{a/b} = 
    \begin{bmatrix}
        \pre[c]\omega_{a/b}(t)\\
        \pre[c]v_{a/b}(t)\\
    \end{bmatrix}

Implementation
==============
See :class:`arboris.RigidMotion` or :mod:`homogeneousmatrix`
