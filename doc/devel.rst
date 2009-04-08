=============
Developpement
=============

This is a small page to help starting using and hacking python-arboris.
  

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
  really simple to use, but with some weird choices (for orientation...). It maybe too simple. Also, it is not really well maintained, documented and distributed (the latest version is not packaged in ubuntu, and is not availble for python2.6 on windows).

blender:
  Blender may be a great way to interact with the simulation, at the price of some redundancy, as it already has its own data structures for kinematics models and so. It is only suited for offline visualization.

blender with game engine:
  ?

`openscenegraph`_:
  Seems great, but lacks well established python bindings. According to Anthony, `http://code.google.com/p/osgswig`_ is usable

mayavi2:
  Joseph did some testing and it appeared that it was incredibly slow. It is based on VTK

`VTK`_:
  Has python bindings.

.. _`visual python`:
  http://vpython.org

.. _`openscengraph`:
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

Install and set up git::

  sudo aptitude install git-gui gitk gitmagic
  git config --global user.name "Your Name Comes Here"
  git config --global user.email you@yourdomain.example.com

Get python-arboris from git (you'll need an ssh account ``yourlastname`` on vizir)::

  cd ~
  git clone yourlastname@vizir.robot.jussieu.fr:/home/seb/pyarboris
  cd pyarboris
  git remote add vizir-seb yourlastname@vizir.robot.jussieu.fr:pyarboris

Now in ``~/pyarboris`` you have all the history of python-arboris. You can get updates with::

  git pull vizir-seb
