========================
Constraints and contacts
========================

Gauss-Seidel algorithm
======================

.. automethod:: arboris.core.World.update_constraints


Constraints
===========

.. currentmodule:: arboris.constraints


Kinematic Constraints
---------------------

.. autoclass:: BallAndSocketConstraint
   :members: update, solve


Contacts
--------

.. note::
   we do not deal with non-point contact

.. autoclass:: PointContact
   :members: update

.. autoclass:: SoftFingerContact
   :members: update, solve



