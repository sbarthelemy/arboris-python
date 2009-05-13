===========
Quick start
===========

A world constists of rigid bodies interconnected by ideal joints in a 
tree topology, additionnal constraints and controllers.


First, let's create a world ``w``, which will consist of the bodies and 
joints which form a human model, plus the ground body.

.. doctest::

  >>> from human36 import human36
  >>> (w, foo, foo2) = human36()
  >>> bodies = w.getbodiesdict()
  >>> bodies['Head'].mass


.. doctest::

  >>> w.update_geometric()
  >>> bodies['HandR'].pose
  >>> bodies['HandR'].jacobian

.. doctest::

  >>> from controllers import WeightController
  >>> c = WeightController(w.ground)
  >>> w.add_controller(c)


  array([ 0.,  0.,  1.,  0.,  0.,  0.])



