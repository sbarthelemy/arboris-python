#!python
import rigidmotion as rm
import numpy as np
import homogeneousmatrix as htr
import adjointmatrix as Ad
import visu as v
from visual import controls
w = rm.World()


Hprev = np.dot(np.eye(4), htr.transl([-4,0,0]))
Hnext = np.dot(np.eye(4), htr.transl([4,0,0]))

for i in range(1,3):
    prev_body = w.bodies[i-1]
    left_frame = prev_body.newframe(Hnext,(prev_body.name +'-next'))
    
    new_body = rm.Body(name=('body%02d' % i))
    right_frame = new_body.newframe(Hprev,(new_body.name +'-prev'))

    joint = rm.HingeJoint(
        name = ('joint%02d' % (i-1)),
        leftframe = left_frame,
        rightframe = right_frame,
        gpos = .5)
    w.addjoint(joint)


H1 = htr.rotx(45.0/180*3.14)
H1i = htr.inv(H1)
Ad1 = htr.adjoint(H1)
Ad1i = htr.adjoint(H1i)
T2 = rm.Transform()
H2 = T2.gethomogeneousmatrix()
h1 = rm.HomogeneousMatrix(H1)
t1 = rm.Twist([0, 0, 1, 0, 0, 0])

w.geometric()
vw = v.World(w)

