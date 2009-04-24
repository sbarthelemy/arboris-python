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
- make a python package (with an __init__.py) instead of a modulde
- generate a package with
  - changelog
  - version numbers
  - tar/deb


Programming
===========

- find a way/class to collect states

  - states are positions and (someteimes) velocities.
  - velocities are already grouped in Wordl._gvel
  - positions might be grouped 

- do the prediction,

  - need predicted bodies poses and twists, no joint-space model (kinematic)

- returns dicts of frames/joints/bodies
- Create a virtual class ``Named`` for ``*._name`` handling ?
- implement a true recursive-newton-euler linearized algorithm?

Name conventions
================

  - rename dynamic() to update_dynamic() (idem for geometric, kinematic...))
  - gvel -> jvel (joint generalized velocities)?
  - find better name for controller_viscosity 
  - ``Body.ancestors()`` => ``.iter_ancestor_bodies()``


Done (and kept for reference)
=============================

- find a better name for ``Body._ref_frame``. Current suggestions:
  
  - ``prev_frame``,
  - ``parent_frame``,
  - ``base_frame``
  - ``frames[0]``

- find a better name for ``Body._new_frame``. Current suggestions:
  
  - ``next_frame``,
  - ``moving_frame``,
  - ``local_frame``,
  - ``frames[1]``


