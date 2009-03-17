#!python
import rigidmotion as rm
import numpy as np
import homogeneousmatrix as htr
import adjointmatrix as Ad
import controlwindow as cw
import visu as v
import visual
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

w.geometric()
vw = v.World(w)


for t in range(100):
    visual.rate(10)
    print t
    w.joints[1].gpos=t/20.
    w.geometric()
    vw.update()
