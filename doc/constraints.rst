========================
Constraints and contacts
========================

Gauss-Seidel algorithm
======================

.. automethod:: arboris.core.World.update_constraints
    :noindex:

Constraints
===========

.. inheritance-diagram:: arboris.constraints

.. currentmodule:: arboris.constraints


Kinematic Constraints
---------------------

.. autoclass:: BallAndSocketConstraint
   :members: update, solve

Joint limits
------------

.. autoclass:: JointLimits

Contacts
--------

.. note::
   we do not deal with non-point contact

.. autoclass:: PointContact
   :members: update

.. autoclass:: SoftFingerContact
   :members: update, solve



