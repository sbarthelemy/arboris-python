================
Quirks
================

This pages illustrates a few quirks and gotchas from python, numpy and arboris.

Python
======

Default values
--------------

The function argument default values ar'e evaluated only once. This makes a difference when the default is a mutable object such as a list, dictionary, or instances of most classes. For instance in this example

.. testcode:: group1

  import numpy as np
  class MyJoint(object):
      def __init__(self, gpos=np.zeros((1))):
          self.gpos = gpos
  j1 = MyJoint()
  j2 = MyJoint()
  j1.gpos[0] = 3.14
  print(j2.gpos)
  print(j1.gpos is j2.gpos)

The ``gpos`` is shared by the two instances ``j1`` and ``j2``:

.. testoutput:: group1

  [ 3.14]
  True

A solution is:

.. testcode:: group2

  import numpy as np
  class MyJoint(object):
      def __init__(self, gpos=None):
          if gpos == None:
              gpos=np.zeros((1))
          self.gpos = gpos
  j1 = MyJoint()
  j2 = MyJoint()
  print(j1.gpos is j2.gpos)

.. testoutput:: group2

  False




Scope and nested functions
--------------------------

The following example illustrates scoping with nested function definitions:
a nested function as read access to (all) its parents namespaces, but attempting to write them creates a new local one. Note that python-3 provides read/write access with the ``nonlocal`` statement.

.. testcode::

  def outer():
      a = 'a defined in outer'
      b = 'b defined in outer'
      c = 'c defined in outer'
      def middle():
          def inner():
              print("inner: {var}".format(var=a))
              print("inner: {var}".format(var=b))
              print("inner: {var}".format(var=c))          
          c = 'c defined in middle'
          print("middle: {var}".format(var=a))
          print("middle: {var}".format(var=b))
          print("middle: {var}".format(var=c))
          # middle as read access to outer()'s namespace
          inner()

      b = 'b re-defined in outer'
      middle()
      print("outer: {var}".format(var=a))
      print("outer: {var}".format(var=b))
      print("outer: {var}".format(var=c))
  outer()

returns

.. testoutput::

  middle: a defined in outer
  middle: b re-defined in outer
  middle: c defined in middle
  inner: a defined in outer
  inner: b re-defined in outer
  inner: c defined in middle
  outer: a defined in outer
  outer: b re-defined in outer
  outer: c defined in outer


Numpy
======

TODO: illustrate views of arrays

Arboris
=======

