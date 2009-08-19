
>>> from arboris.core import World, Body
>>> from arboris.joints import *
>>> from numpy import dot
>>> from numpy.linalg import norm

>>> world = World()
>>> Ba = Body()
>>> Rzyx = RzRyRxJoint(frames=(world.ground, Ba))
>>> Bzy = Body(name='zy')
>>> Byx = Body(name='yx')
>>> Bb = Body()
>>> Rz = RzJoint(frames=(world.ground,Bzy))
>>> Ry = RyJoint(frames=(Bzy,Byx))
>>> Rx = RxJoint(frames=(Byx,Bb))
>>> world.init()
>>> (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
>>> Rzyx.gpos[:] = (az, ay, ax)
>>> (Rz.gpos[0], Ry.gpos[0], Rx.gpos[0]) = (az, ay, ax)
>>> world.update_dynamic()
>>> Ja = Ba.jacobian[:,0:3]
>>> Jb = Bb.jacobian[:,3:6] 
>>> norm(Ja-Jb) < 1e-10
True


