========================
Constraints and contacts
========================

Gauss-Seidel algorithm
======================

.. automethod:: arboris.World.update_constraints


Constraints
===========

.. currentmodule:: constraints


Kinematic Constraints
---------------------

.. autoclass:: BallAndSocketConstraint
   :members: update, solve


Contact Constraints
-------------------

.. note::
   we do not deal with non-point contact

.. autoclass:: PointContact
   :members: update

.. autoclass:: SoftFingerContact
   :members: update, solve



