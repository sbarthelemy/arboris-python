====================
Some things to do...
====================

- Add copyright
- Add GPL
- Publish (how? where? github? vizir?)

  - git-web
  - http://www.kernel.org/pub/software/scm/git/docs/howto/setup-git-server-over-http.txt
  - http://scie.nti.st/2007/11/14/hosting-git-repositories-the-easy-and-secure-way

- Move from make to scons
- generate a package with
  - changelog
  - version numbers
  - tar/deb

- Add world as an argument to human36 and triplehinge
- naming conventions
  - name of Body.frame[0]
  - rename pose() to {rel,local,body}_pos() and abs/world_pose()?
  - find a better name for ``Body._ref_frame`` (current suggestions:
            ``prev_frame``, ``parent_frame``, ``base_frame``)
        TODO: find a better name for ``Body._new_frame`` (current suggestions:
        ``next_frame``, ``moving_frame``, ``local_frame``)
  - replace ``ref_frame``, ``new_frame`` by a ``frames`` tuple or dict?
  - rename dynamic() to update_dynamic() (idem for geometric, kinematic...))
  - implement a true recursive-newton-euler linearized algorithm?
  - gvel -> jvel (joint generalized velocities)?
