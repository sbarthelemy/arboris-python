=============
Development
=============

This is a small page to help hacking arboris-python.
  
Using Git
=========

Linux setup
-----------

Install and set up git::

  sudo aptitude install git-gui gitk
  git config --global user.name "Your Name Comes Here"
  git config --global user.email you@yourdomain.example.com
  git config --global --add color.diff always
  git config --global --add color.interacive always

Then, run ``git help tutorial`` for help.

Workflow
--------

Seb, as the first author of arboris, creates a repository on its laptop::

  seb@seb-laptop$ mkdir arboris-python
  seb@seb-laptop$ cd arboris-python
  seb@seb-laptop$ git init
  seb@seb-laptop$ edit ...files... 
  seb@seb-laptop$ git add ...files...
  seb@seb-laptop$ git commit

Then, in order to make the repository accessible to others, Seb puts it on the vizir server::

  seb@seb-laptop$ scp -r  arboris-python seb@vizir.robot.jussieu.fr:
  seb@seb-laptop$ ssh seb@vizir.robot.jussieu.fr
  seb@vizir$ git clone --bare arboris-python arboris-python.git
  seb@vizir$ rm -rf arboris-python
  seb@vizir$ cd arboris-python.git
  seb@vizir$ git config receive.denyNonFastforwards true
  seb@vizir$ logout
  seb@seb-laptop$ rm -rf arboris-python
  seb@seb-laptop$ git clone ssh://seb@vizir.robot.jussieu.fr/home/seb/arboris-python.git

Now, Seb can work locally and push back to vizir::

  TODO: explain how

Joe wants to help. He can fetch Seb's repository, and produce a patch::

  joe@joe-laptop$ git clone ssh://salini@vizir.robot.jussieu.fr/home/seb/arboris-python.git
  joe@joe-laptop$ cd arboris-python
  joe@joe-laptop$ edit ...files... #(Joe improves the visualization)
  joe@joe-laptop$ git add ...files...
  joe@joe-laptop$ git commit
  joe@joe-laptop$ git diff master..origin/master > visu-impr.patch

Then he sends the patch to Seb by email, who applies it and push the result back to vizir::

  seb@seb-laptop$ git apply visu-impr.patch
  seb@seb-laptop$ git add ...files...
  seb@seb-laptop$ git commit 
  seb@seb-laptop$ git push 

Eventually, when Joe issues a new pull, everything gets merged gracefully::

  joe@joe-laptop$ git pull


Design choices
==============

Visualization tools
-------------------

We'd like to support both interactive and offline visualization. We'd like it to be simple enough for a user to add custom shapes and powerful enough to import and animate complex graphics. It should work on linux, mac OS and Windows computers. We've considered these candidates:

`Visual python <http://vpython.org>`_: 
  visual python is very simple to use but does not seem activelly maintained, documented nor distributed (the latest version is not packaged in ubuntu, and is not available for python2.6 on windows). The users community seems small too. This `unofficial documentation in French <ftp://ftp-developpez.com/guigui/cours/python/vpython/fr/ManuelVpython.pdf>`_ may be worth a read.
  
blender:
  Blender may be a great way to interact with the simulation. Joseph succeeded in generating programmatically a skeletton (Armature), and feeding it with generalized coordinates trajectories. However,
 
  - it is only suited for offline visualization (see blender game engine for an alternative), 
  - it comes at the price of some redundancy (which may lead to confusion), as blender has its own data structures for kinematics models.

blender with game engine:
  Not really tried yet

`OpenSceneGraph <http://www.openscenegraph.org>`_:
  Seems great, but lacks well established python bindings. We tried the `osgswig <http://code.google.com/p/osgswig>`_ with OpenSceneGrap 2.6, it works fine in spite of the hundreds of warning during the compilation. Version 2.8 adds support for character animation, we should have a look.

`VTK <http://www.vtk.org>`_:
  A quite famous visualization toolkit in C++, which has python bindings.

`TVTK <https://svn.enthought.com/enthought/wiki/TVTK>`_:
  Another layer of wrappers around the VTK python bindings.

`MayaVi2 <https://svn.enthought.com/enthought/wiki/MayaVi>`_:
  A visualization framework built around TVTK. It is more suited to vector fields visualization than to scene viewing. 

`matplotlib <http://matplotlib.sourceforge.net/>`_:
  A 2D viewing library, which used to have limited 3D support. The 3D part was eventually removed.
