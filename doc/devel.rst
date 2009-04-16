=============
Developpement
=============

This is a small page to help using and hacking python-arboris.
  

What we use...
==============

...for the simulation
---------------------

All the program is written in python2.6. See Python26Doc_ (or Python26LocalDoc_) and start with the tutorial. We also use numpy. See NumpyDoc_. Numpy supports n-dimensionnal arrays (class :class:`numpy.ndarray`) and matrices (built with `numpy.matrix`) which are 2-d matrices with special behavior, similar to the matlab one. In pyarboris we never uses matrices.

.. _Python26Doc:
  http://docs.python.org/

.. _Python26LocalDoc:
  file:///usr/share/doc/python2.6-doc/html/index.html

.. _NumpyDoc:
  http://docs.scipy.org/doc/


...for the documentation
------------------------

The doc is written in the reST markup language and processed by sphinx (version >=5). See the sphinx (and reST) documentation at SphinxDoc_ (or SphinxLocalDoc_). We use sphinx plugins which allow to embed latex in the doc, to parse the docstrings spread in the code and to run the examples with doctest.

.. _SphinxDoc:
  http://sphinx.pocoo.org/

.. _SphinxLocalDoc:
  file:///usr/share/doc/python-sphinx/html/index.html



...for the visualization
------------------------

No real choice has been done yet.

We'd like to support both interactive and offline vizualisation. We'd like it to be simple enough for a user to add custom shapes and complete enough to import and animate complex graphics. It should work on linux, mac OS and Windows computers.

Here is a list of candidates:

`visual python`_: 
  Very simple to use, it may even be too simple. Also, it is not really well maintained, documented nor distributed (the latest version is not packaged in ubuntu, and is not availble for python2.6 on windows). This unofficial documentation in French may be worth a read: _`VpythonDocFr`.
  
blender:
  Blender may be a great way to interact with the simulation. Joseph succeeded in generating programmatically a skeletton (Armature), and feeding it with generalized coordinates trajectories. However,
 
  - it is only suited for offline visualization (see blender game engine for an alternative), 
  - it comes at the price of some redundancy (which may lead to confusion), as blender has its own data structures for kinematics models and so...

blender with game engine:
  ?

`openscenegraph`_:
  Seems great, but lacks well established python bindings. According to Anthony, `http://code.google.com/p/osgswig`_ is usable though.

`VTK`_:
  Has python bindings.

TVTK_:
  ?

mayavi2:
  Joseph did some testing and it appeared that it was incredibly slow. It is based on VTK


.. _DocVpythonDocFr:
  ftp://ftp-developpez.com/guigui/cours/python/vpython/fr/ManuelVpython.pdf
 
.. _`visual python`:
  http://vpython.org

.. _openscenegraph:
  http://www.openscenegraph.org

.. _`VTK`:
  http://www.vtk.org

Set up for Ubuntu Jaunty
========================

We develop using a (possibly virtual) computer with ubuntu jaunty. In such a case, the following commands shoud set up the development environment ::
  sudo aptitude install python2.6-doc python-sphinx python-numpy ipython

Install python-visual (build from sources)::

  sudo aptitude install visual-deps... #TODO
  cd ~
  wget visual... #TODO
  tar visual... #TODO
  mkdir usr
  cd visual
  ./configure --prefix=$HOME/usr
  make
  make install
  echo "export PYTHONPATH=~/usr/lib/python2.6:~/usr/lib/python2.6/dist-packages" >> ~/.bashrc
  exec bash

For troubleshooting the compilation process, see ``src/build.log``.

Using Git
=========

Linux setup
-----------

Install and set up git::

  sudo aptitude install git-gui gitk gitmagic
  git config --global user.name "Your Name Comes Here"
  git config --global user.email you@yourdomain.example.com

Then, issue ``git help tutorial`` for help or look at ``/usr/share/doc/gitmagic/html``.

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

Joe wants to help. He can send patches to Seb by email::

  joe@joe-laptop$ git clone ssh://salini@vizir.robot.jussieu.fr/arboris-python.git
  joe@joe-laptop$ cd arboris-python
  joe@joe-laptop$ edit ...files... #(improve vizualisation by adding labels)
  joe@joe-laptop$ git add ...files...
  joe@joe-laptop$ git commit
  joe@joe-laptop$ git diff > labels-in-vizu.patch


Future
======

There are to do items spread overall the code and the documentation, ``grep  TODO {src,doc}/*{.py,.rst}`` should get them
