=======
Install
=======


What we use...
==============

...for the simulation
---------------------

The program is written in python 2.6. See the documentation 
`online <http://docs.python.org/>`_
(or `locally <file:///usr/share/doc/python2.6-doc/html/index.html>`_
on linux)
and start with the tutorial. 

We also use numpy. See 
`the numpy documentation <http://docs.scipy.org/doc/>`_ and 
`NumPy for Matlab users <http://www.scipy.org/NumPy_for_Matlab_Users>`_. 

Numpy supports n-dimensionnal arrays (class :class:`numpy.ndarray`) and 
matrices (built with ``numpy.matrix``) which are 2-d matrices with special 
behavior, similar to the matlab one. In arboris-python we never uses 
matrices, they are evil.

We also optionally use `h5py <http://h5py.alfven.org/>`_ to store 
simulation results in the `HDF5 format <http://www.hdfgroup.org/HDF5/>`_.


...for the documentation
------------------------

The documentation is written in the reST markup language and processed 
by sphinx (version >=6). See the sphinx (and reST) documentation 
`online <http://sphinx.pocoo.org/>`_ 
(or `locally <file:///usr/share/doc/python-sphinx/html/index.html>`_ 
on linux). We use sphinx plugins
which allow to embed latex in the doc, to parse the docstrings spread 
in the code and to run the examples with doctest. To generate the uml 
diagrams, graphviz is needed too.


...for the visualization
------------------------

We use `OpenSceneGraph <http://www.openscenegraph.org>`_, 
through the `osgswig <http://code.google.com/p/osgswig>`_ bindings.

On linux, we use OSG version 2.8. On Windows, we use version 2.6 because 
there is no installer for the 2.8 series.


Installing Python, Numpy and IPyton
===================================

...for Ubuntu Jaunty & Karmic
-----------------------------

Install the packaged stuff::

  sudo aptitude install python2.6-doc python-numpy ipython

...for Windows
--------------

Install...

- python 2.6 from http://www.python.org/download/. The current installer 
  is named "Python 2.6.3 Windows installer".
- numpy from http://numpy.scipy.org/. Ensure to choose a version 
  compatible with python 2.6. The current installer is named 
  "numpy-1.3.0-win32-superpack-python2.6.exe".
- pyreadline from http://ipython.scipy.org/moin/PyReadline/Intro
- IPython from http://ipython.scipy.org


Installing OpenSceneGraph 2.8
=============================

...for Ubuntu Karmic
--------------------

OpenSceneGraph 2.8 is packaged::

  sudo aptitude install libopenscenegraph-dev openscenegraph openscenegraph-doc

.. note::
  It seems that osgswig is incompatible with the version shiped with 
  ubuntu (2.8.1-1). You might install the 2.8.2 from source instead, 
  as explained in the following section.

...for Ubuntu Jaunty
--------------------

OpenSceneGraph 2.8 is not packaged.

Install OpenSceneGraph 2.8.2 from source::

  sudo aptitude install wx-common libwxgtk2.8-dev #TODO: not sure this is useful
  svn export http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-2.8.2
  cd OpenSceneGraph-2.8.2
  mkdir build
  cd build
  ccmake ..

Then check the option to compile the wrappers (``BUILD_OSG_WRAPPERS``), 
compile and install (warning, the compilation takes lots of memory and time)::

  make
  sudo make install
  TODO: how to check everything was fine?

...for Windows
--------------

It will be installed together with osgswig, see next section.


Installing osgswig
==================

...for Unbuntu Jaunty & Karmic
------------------------------

Install OpenSceneGraph python bindings from sources (inspired by 
`this wiki page <http://code.google.com/p/osgswig/wiki/BuildInstructions>`_)::

  sudo aptitude install swig
  svn checkout -r207 http://osgswig.googlecode.com/svn/trunk/ osgswig
  cd osgswig
  mkdir build
  cd build
  ccmake .. -DCMAKE_BUILD_TYPE=Release
  make
  cp -r lib/python/osgswig-0.9.1/* /home/seb/.local/lib/python2.6/site-packages/

Don't worry about the hundreds of warnings during the compilation.

...for Windows
--------------

Install osgswig from http://code.google.com/p/osgswig/. The current 
installer is named "osgPython-2.6.1-0-py26.exe". The install process is 
detailed `here <http://code.google.com/p/osgswig/wiki/InstallationWindows>`_.


Installing h5py
===============

...for ubuntu jaunty and karmic
-------------------------------

Install hdf5 library and headers::

    sudo aptitude install libhdf5-dev

Download, compile and install the python bindings (h5py)::

    wget http://h5py.googlecode.com/files/h5py-1.2.1.tar.gz
    tar -xzf h5py-1.2.1.tar.gz
    cd h5py-1.2.1
    python setup.py build
    python setup.py install --prefix=/home/seb/.local


Installing cvxmod
=================

...for ubuntu karmic
--------------------

install cvxopt from ubuntu and cvxmod from sources::

  sudo aptitude install python-cvxopt
  cd /tmp/
  wget http://cvxmod.net/dist/cvxmod-0.4.6.tar.gz
  tar xzf cvxmod-0.4.6.tar.gz
  cd cvxmod-0.4.6/
  sudo python setup.py install

...for Windows
--------------

cvxmod is available as an installer for windows and python 2.6. However,
it depends on cvxopt, which is only has a Windows installer for 
python2.5. We have no solution for this yet.

Note that cvxmod and cvxopt are not required by arboris itself but by 
a controller, which you may not need.


Installing arboris-python
=========================

...for Ubuntu
-------------

unzip, go in the new directory, then run::

  sudo python setup.py install 

or, if you prefer to install it in your home::

  python setup.py install --user

...for Windows
--------------

unzip, go in the new directory, then run::

  C:\python26\python.exe setup.py install
