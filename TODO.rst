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

- fix program version in sphinx doc & setup.py
- compile the doc from setup.py

Programming
===========

Small changes
-------------

- find a way/class to collect states

  - states are positions and (sometimes) velocities.
  - velocities are already grouped in Wordl._gvel
  - positions might be grouped 

- create functions to merge bodies/worlds etc.
- implement a true recursive-newton-euler linearized algorithm?

Contacts
--------

- understand why contacts are bumping
- add sliding friction law

Dynamic
-------

- document dynamics
- thorougly test dynamical model on a tree-like robot
- check results against HuMAnS
- support kinematic (stateless) joints

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

Visu_osg
--------

- add more colors for bodies


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
