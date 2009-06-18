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

- get version/release number from git tags


Documentation
=============

- replace every \Hg by \H
- migrate to sphinx 0.6, to benefit from graphviz class diagrams
- make an example showing energy evolution
- make an example showing computational singularity


Programming
===========

Small changes
-------------

- find a way to collect states

  - states are positions and (sometimes) velocities.
  - velocities are already grouped in Wordl._gvel
  - positions might be grouped 

- find a safe way to set states/pos/velocity (better than joint.gvel[0] = 2.)
- create functions to merge bodies/worlds etc.
- implement a true recursive-newton-euler linearized algorithm?
- add visco-elastic joints
- clean the cython mess (remove all or at least provide a .py equivalent)

Joints Limits
-------------

merge constraints.JointLimits with joints.LinearConfigurationSpaceJoint ?

pro:

- easier for user
- no more need for initjointspace()
- fewer names for the user
- in real world, joints work that way

con: 

- yet another multiple inheritance
- the two algorithm are distincts

Contacts
--------

- understand why contacts are bumping
- add sliding friction law
- improve the stop condition of the Gauss-Seidel algorithm

Dynamic
-------

- thorougly test dynamical model on a tree-like robot
- check results against HuMAnS
- support kinematic (stateless) joints

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
