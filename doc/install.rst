=======
Install
=======


What we use...
==============

...for the simulation
---------------------

The program is written in python 2.6. See the documentation
`online <http://docs.python.org/>`__
(or `locally <file:///usr/share/doc/python2.6-doc/html/index.html>`__
on linux)
and start with the tutorial.

We also use numpy. See
`the numpy documentation <http://docs.scipy.org/doc/>`_ and
`NumPy for Matlab users <http://www.scipy.org/NumPy_for_Matlab_Users>`_.

Numpy supports n-dimensionnal arrays (class :class:`numpy.ndarray`) and
matrices (built with ``numpy.matrix``) which are 2-d matrices with special
behavior, similar to the matlab one. In arboris-python we never uses
matrices, they are evil.

We also use `h5py <http://h5py.alfven.org/>`_ to store simulation
results in the `HDF5 format <http://www.hdfgroup.org/HDF5/>`_.


...for the documentation
------------------------

The documentation is written in the reST markup language and processed
by sphinx (version >=6). See the sphinx (and reST) documentation
`online <http://sphinx.pocoo.org/>`__
(or `locally <file:///usr/share/doc/python-sphinx/html/index.html>`__
on linux). We use sphinx plugins which allow

- to embed latex in the doc,
- to parse the docstrings spread in the code,
- to run the examples with doctest,
- to generate the uml diagrams using graphviz.


...for the visualization
------------------------

We use the `OpenSceneGraph <http://www.openscenegraph.org>`_ library,
through the `osgswig <http://code.google.com/p/osgswig>`_ bindings.

On linux, we use OSG version 2.8. On Windows, we use version 2.6 because
there is no installer for the 2.8 series.


Installation on Lucid Lynx (10.04)
==================================

Installing Python, Numpy and IPython
------------------------------------

Install the packaged stuff::

  sudo aptitude install python2.6-doc ipython2.6 \
                        python-numpy python-numpy-doc \
                        ipython python-h5py


Installing arboris-python
-------------------------

unzip, go in the new directory, then run::

  python setup.py install --user

Installing osgswig
------------------

OpenSceneGraph build dependancies::

  sudo aptitude build-depends libopenscenegraph-dev
  sudo aptitude install subversion build-essential cmake

Get the code source and compile it (compilation is quite long)::

  svn co \
  http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-2.9.7
  cd OpenSceneGraph-2.9.7
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
           -DCMAKE_CXX_FLAGS=-D__STDC_CONSTANT_MACROS \
           -DBUILD_OSG_WRAPPERS=ON \
           -DBUILD_OSG_APPLICATIONS=OFF
  make
  sudo make install

OSG libraries were installed in ``/usr/local``. Ensure that ``ld`` will find
them::

  echo $LD_LIBRARY_DIR

OsgSwig dependancies::

  sudo aptitude install swig python-dev

Install the python bindings from sources (inspired by
`this wiki page <http://code.google.com/p/osgswig/wiki/BuildInstructions>`_)::

  cd /tmp
  svn checkout http://osgswig.googlecode.com/svn/trunk/ osgswig
  cd osgswig
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make
  cp  lib/python/osgswig-0.9.1/* ~/.local/lib/python2.6/site-packages/

Don't worry about the hundreds of warnings during the compilation.
(It worked with revision 227 of osgswig).

Installing cvxmod
-----------------

Install cvxopt from ubuntu and then cvxmod from sources::

  sudo aptitude install python-cvxopt
  cd /tmp
  wget http://cvxmod.net/dist/cvxmod-0.4.6.tar.gz
  tar xzf cvxmod-0.4.6.tar.gz
  cd cvxmod-0.4.6/
  python setup.py install --prefix=~/.local

Installing the tools needed to build the documentation
------------------------------------------------------

::

  sudo aptitude install python-sphinx graphviz texlive-latex-base dvipng

For Mac OS 10.6 (Snow Leopard)
==============================

Installing dependancies
-----------------------

Mac OS ships with python 2.5, 2.6 and numpy pre-installed. The other
packages can be easily installed using  `macports <http://www.macports.org>`_.

::

    sudo port install py26-h5py py26-ipython py26-matplotlib osgswig-devel python-cvxmod

Installing arboris-python
-------------------------

unzip, go in the new directory, then run::

  python2.6 setup.py install --user


For windows
===========

Installing Python, Numpy, IPython and h5py
------------------------------------------

...manually
~~~~~~~~~~~

Install...

- python 2.6 from http://www.python.org/download/. The current installer
  is named "Python 2.6.3 Windows installer".
- numpy from http://numpy.scipy.org/. Ensure to choose a version
  compatible with python 2.6. The current installer is named
  "numpy-1.3.0-win32-superpack-python2.6.exe".
- pyreadline from http://ipython.scipy.org/moin/PyReadline/Intro
- IPython from http://ipython.scipy.org
- h5py from http://code.google.com/p/h5py/downloads/list

...from Python(x,y)
~~~~~~~~~~~~~~~~~~~

All these programs (and many others) are conveniently packaged by the
`Python(x,y) <http://www.pythonxy.com>`_ project, you may install them
from there.


Installing arboris-python
-------------------------

unzip, go in the new directory, then run::

  C:\python26\python.exe setup.py install


Installing osgswig and OpenSceneGraph
-------------------------------------

Install osgswig from http://code.google.com/p/osgswig/. The current
installer is named "osgPython-2.6.1-0-py26.exe". The install process is
detailed `here <http://code.google.com/p/osgswig/wiki/InstallationWindows>`_.


Installing cvxmod
-----------------

No Windows installer compatible with python 2.6 is available on
`cvxopt's website <http://abel.ee.ucla.edu/cvxopt>`_ yet. However,
one is available from another place:
http://abel.ee.ucla.edu/smcp/download/smcp-0.2a.win32-py2.6.zip

Then install cvxmod from http://cvxmod.net/install.html

