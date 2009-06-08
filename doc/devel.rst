=============
Development
=============

This is a small page to help using and hacking arboris-python.
  

What we use...
==============

...for the simulation
---------------------

The program is written in python 2.6 and in cython. Cython is a language which extends python with typed variables in order to allow C-code generation (and thus faster execution). See Python26Doc_ (or Python26LocalDoc_) and start with the tutorial. We also use numpy. See `the numpy documentation <http://docs.scipy.org/doc/>`_ and `NumPy_for_Matlab_Users <http://www.scipy.org/NumPy_for_Matlab_Users>`_. Numpy supports n-dimensionnal arrays (class :class:`numpy.ndarray`) and matrices (built with `numpy.matrix`) which are 2-d matrices with special behavior, similar to the matlab one. In arboris-python we never uses matrices. 

.. _Python26Doc:
  http://docs.python.org/

.. _Python26LocalDoc:
  file:///usr/share/doc/python2.6-doc/html/index.html


...for the documentation
------------------------

The doc is written in the reST markup language and processed by sphinx (version >=5). See the sphinx (and reST) documentation `online <http://sphinx.pocoo.org/>`_ (or `locally <file:///usr/share/doc/python-sphinx/html/index.html>`_ if you followed the ubuntu setup instructions). We use sphinx plugins which allow to embed latex in the doc, to parse the docstrings spread in the code and to run the examples with doctest. To convert some pictures from svg, inskcape should also be installed and accessible through the command line.

...for the visualization
------------------------

No real choice has been done yet.

We'd like to support both interactive and offline vizualisation. We'd like it to be simple enough for a user to add custom shapes and powerful enough to import and animate complex graphics. It should work on linux, mac OS and Windows computers. We've considered these candidates:

`Visual python <http://vpython.org>`_: 
  visual python is very simple to use but does not seem activelly maintained, documented nor distributed (the latest version is not packaged in ubuntu, and is not available for python2.6 on windows). The users community seems small too. This `unofficial documentation in French <ftp://ftp-developpez.com/guigui/cours/python/vpython/fr/ManuelVpython.pdf>`_ may be worth a read.
  
blender:
  Blender may be a great way to interact with the simulation. Joseph succeeded in generating programmatically a skeletton (Armature), and feeding it with generalized coordinates trajectories. However,
 
  - it is only suited for offline visualization (see blender game engine for an alternative), 
  - it comes at the price of some redundancy (which may lead to confusion), as blender has its own data structures for kinematics models.

blender with game engine:
  Not really tried yet

`OpenSceneGraph <http://www.openscenegraph.org>`_:
  Seems great, but lacks well established python bindings. We tried the `osgswig <http://code.google.com/p/osgswig>`_ with OpenSceneGrap 2.6, it works fine in spite of the hundreds of warning during the compilation.

`VTK <http://www.vtk.org>`_:
  A quite famous visualisation toolkit in C++, which has python bindings.

`TVTK <https://svn.enthought.com/enthought/wiki/TVTK>`_:
  Another layer of wrappers around the VTK python bindings.

`MayaVi2 <https://svn.enthought.com/enthought/wiki/MayaVi>`_:
  A visualisation framework built around TVTK. It is more suited to vector fields visualisation than to scene viewing. 

`matplotlib <http://matplotlib.sourceforge.net/>`_:
  A 2D viewing library, which used to have limited 3D support. The 3D part was eventually removed.
  

Setting up...
=============

...for Ubuntu Jaunty
--------------------

Install the packaged stuff::

  sudo aptitude install python2.6-doc python-sphinx python-numpy ipython

Install OpenSceneGraph 2.6 from source::

  sudo aptitude install wx-common libwxgtk2.8-dev #TODO: not sue this is useful
  svn export http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-2.6.1
  cd OpenSceneGraph-2.6.1
  mkdir build
  cd build
  ccmake ..

Then check the option to compile the wrappers, then::

  make
  sudo make install
  TODO: how to check everything was fine?

Install OpenSceneGraph python bindings from souces (inspired by `this wiki page <http://code.google.com/p/osgswig/wiki/BuildInstructions>`_)::

  sudo aptitude install swig
  svn checkout http://osgswig.googlecode.com/svn/trunk/ osgswig
  cd osgswig
  mkdir build
  cd build
  ccmake .. -DCMAKE_BUILD_TYPE=Release
  make
  cp -r lib/python/ ~/.local/lib/python2.6/site-packages/OpenSceneGraph
  touch ~/.local/lib/python2.6/site-packages/OpenSceneGraph/__init__.py

Install python-visual from sources::

  sudo aptitude install visual-deps... #TODO
  cd ~
  wget visual... #TODO
  tar visual... #TODO
  mkdir usr
  cd visual
  ./configure --prefix=$HOME/usr # /!\TODO install in .local/lib/python2.6
  make
  make install
  echo "export PYTHONPATH=~/usr/lib/python2.6:~/usr/lib/python2.6/dist-packages" >> ~/.bashrc
  exec bash

For troubleshooting the compilation process, see ``src/build.log``.

...for Windows
--------------

Install...

- python 2.6 from http://www.python.org/download/. The current installer is named "Python 2.6.2 Windows installer".
- numpy from http://numpy.scipy.org/. Ensure to choose a version compatible with python 2.6. The current installer is named "numpy-1.3.0-win32-superpack-python2.6.exe".
- osgswig from http://code.google.com/p/osgswig/. The current installer is named "osgPython-2.6.1-0-py26.exe"
- ipython...


Using Git
=========

Linux setup
-----------

Install and set up git::

  sudo aptitude install git-gui gitk
  git config --global user.name "Your Name Comes Here"
  git config --global user.email you@yourdomain.example.com

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
  seb@seb-laptop$ git clone ssh://seb@vizir.robot.jussieu.fr/arboris-python.git

Now, Seb can work locally and push back to vizir::

  TODO: explain how

Joe wants to help. He can fetch Seb's repository, and produce a patch::

  joe@joe-laptop$ git clone ssh://salini@vizir.robot.jussieu.fr/arboris-python.git
  joe@joe-laptop$ cd arboris-python
  joe@joe-laptop$ edit ...files... #(Joe improves the visualisation)
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


Debugging
=========

with IPython
------------

A quite easy way to debug without breakpoints is to embed an ipython shell in te program. For instance, the following program computes an IK notion::

  TODO

You can examine the context at t==10::

        if 10< t <= 11:
            from IPython.Shell import IPShellEmbed
            ipshell = IPShellEmbed()
            ipshell()

with winpdb
-----------


Future
======

There are to do items spread allover the code and the documentation, ``grep  TODO {src,doc}/*{.py,.rst}`` should get them. You might also look at the file ``TODO.txt``
