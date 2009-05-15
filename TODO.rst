====================
Some things to do...
====================

Miscalleneous
=============

- Add copyright
- Add GPL
- Publish (how? where? github? vizir?)

  - git-web
  - http://www.kernel.org/pub/software/scm/git/docs/howto/setup-git-server-over-http.txt
  - http://scie.nti.st/2007/11/14/hosting-git-repositories-the-easy-and-secure-way

Packaging
=========

- Move from make to setuptools or scons
- make a python package (with an __init__.py) instead of a module
- fix program version in sphinx doc
- generate a package with
  - changelog
  - version numbers
  - tar/deb


Programming
===========

Small changes
-------------

- find a way/class to collect states

  - states are positions and (someteimes) velocities.
  - velocities are already grouped in Wordl._gvel
  - positions might be grouped 

- create functions to merge bodies/worlds etc.
- implement a true recursive-newton-euler linearized algorithm?

Controller interface
--------------------

Most users will interact with arboris through controllers. Current controller 
API is quite bad, for several reasons:

- we have to give ``joints`` argument two times::

    c0 = ProportionalDerivativeController(w.joints[0:3], Kp, Kd, gpos_des)
    w.add_jointcontroller(c0, w.joints[0:3])

- ``joint._dof`` is initialized by  ``w.add_jointcontroller`` and is not 
  available at the joint ``__init__()``
- currently, we have too much or too litte encapsulation
- the controller does not know the world


A solution might be to remove the encapsulation: every controller would produce gforces and viscosity for every joint. (controllers would stil need to know w.ndof.) We could additionnaly provide an "encapsulator" controller class which would do the encapsulation.

We could also give to the controller views of the global gforce and viscosity that it would update.

Frame interface
---------------

I'm not happy with the current Body.frame[0] thing. Frame[0].bpose is pointles and may lead to errors if one user changes it.

A solution might be to: 
- make the :class:`Frame` class astract
- make :class:`Body` class inherit from it 
- the current ``Body.frame[1:]`` would then be of another class (say SubFrame) which also inherits from the :class:`Frame` class

Naming conventions
------------------

  - find better name for controller_viscosity 
  - ``Body.ancestors()`` => ``.iter_ancestor_bodies()``


Done (and kept for reference)
=============================

- find a better name for ``Body._ref_frame``. Current suggestions:
  
  - ``prev_frame``,
  - ``parent_frame``,
  - ``base_frame``,
  - ``proximal_frame``,
  - ``frames[0]`` (the one we chose)

- find a better name for ``Body._new_frame``. Current suggestions:
  
  - ``next_frame``,
  - ``moving_frame``,
  - ``local_frame``,
  - ``distal_frame``,  
  - ``frames[1]`` (the one we chose)

Dreams
======

- add dpose() to RigidMotion
- add support for coupled joints
- add support for non-holonomic joints
