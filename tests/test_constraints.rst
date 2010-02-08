
Test joint limits
=================

>>> from numpy import arange
>>> from arboris.core import simplearm
>>> from arboris.core import simulate
>>> from arboris.constraints import JointLimits
>>> from arboris.controllers import WeightController
>>> world = simplearm()
>>> a = WeightController()
>>> world.register(a)
>>> joints = world.getjoints()
>>> c = JointLimits(joints['Shoulder'], -3.14/2, 3.14/2)
>>> world.register(c)
>>> joints['Shoulder'].gpos[0] = 3.14/2 - 0.1
>>> time = arange(0., 0.1, 1e-3)
>>> simulate(world, time)
>>> print (3.14/2 >= joints['Shoulder'].gpos[0])
True
>>> joints['Shoulder'].gpos[0] = -3.14/2 + 0.1
>>> time = arange(0., 0.1, 1e-3)
>>> simulate(world, time)
>>> print (-3.14/2 <= joints['Shoulder'].gpos[0])
True

Test ball and socket constraint
===============================

>>> from numpy import eye
>>> from arboris.core import Body, World
>>> from arboris.joints import FreeJoint
>>> b0 = Body(mass = eye(6))
>>> w = World()
>>> w.add_link(w.ground, FreeJoint(), b0)
>>> b1 = Body(mass = eye(6))
>>> w.add_link(b0, FreeJoint(), b1)
>>> w.init()
>>> from arboris.controllers import WeightController
>>> ctrl =  WeightController()
>>> w.register(ctrl)
>>> from arboris.constraints import BallAndSocketConstraint 
>>> c0 = BallAndSocketConstraint(frames=(w.ground, b0))
>>> w.register(c0)
>>> w.init()
>>> w.update_dynamic()
>>> dt = 0.001
>>> w.update_controllers(dt)
>>> w.update_constraints(dt)
>>> c0._force
array([ 0.  ,  9.81,  0.  ])
>>> w.integrate(dt)
>>> w.update_dynamic()
>>> b0.pose
array([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          0.00000000e+00],
       [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00,
         -3.09167026e-22],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
          0.00000000e+00],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])
