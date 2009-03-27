=============
Developpement
=============

This is a small page to help starting using and hacking python-arboris.
  

What we use
===========

All the program is written in python2.6. See Python26LocalDoc_ and start with the tutorial. We also use numpy. See NumpyDoc_. Numpy supports n-dimensionnal arrays (class :class:`numpy.ndarray`) and matrices (built with `numpy.matrix`) which are 2-d matrices with special behavior, similar to the matlab one. In pyarboris we never uses matrices.


The doc is written in the reST markup language and processed by sphinx (version >=5). See the sphinx (and reST) documentation at SphinxLocalDoc_. We use sphinx plugins which allow to embed latex in the doc, to parse the docstrings spread in the code and to run the examples with doctest.

.. _Python26LocalDoc:
    file:///usr/share/doc/python2.6-doc/html/index.html

.. _SphinxLocalDoc:
    file:///usr/share/doc/python-sphinx/html/index.html

.. _NumpyDoc:
    http://docs.scipy.org/doc/


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
  ./configure --prefix=~/usr
  make
  make install
  echo "export PYTHONPATH=~/usr/lib/python2.6:~/usr/lib/python2.6/dist-packages" >> ~/.bashrc
  exec bash


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
