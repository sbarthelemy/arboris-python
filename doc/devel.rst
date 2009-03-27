=============
Developpement
=============

This is a small page 

Set up for Ubuntu Jaunty
========================

We develop using a (possibly virtual) computer with ubuntu jaunty. In such a case, the following commands shoud set up the developpment envirronment ::

  sudo aptitude install python2.6-doc python-sphinx python-numpy

  # Install visual dependencies
  sudo aptitude install visual deps
  cd ~
  wget visual... #TODO
  tar visual
  mkdir usr
  cd visual
  ./configure --prefix=~/usr
  make
  make install
  # make python find visual module
  cat "PYTHONPATH=$PYTHONPATH:..." >> ~/.bashrc

  # install and set up git
  sudo aptitude install git
  git ...TODO
  sudo aptitude install meld
  

What we use
===========

All the program is written in python2.6. See Python26LocalDoc_ and start with the tutorial.

We also use numpy. See NumpyLocalDoc_. Numpy support n-dimensionnal arrays



The doc is written in reST and processed by sphinx (version >=5). See the sphinx documentation 

.. Python26LocalDoc:
    /usr/share/doc/python2.6-doc/html/index.html




