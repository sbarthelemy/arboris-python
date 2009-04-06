# coding=utf-8
#!python
import arboris as arb
from arboris import *
import numpy as np
import homogeneousmatrix as htr

import visu_vpython as vpy

#creation du monde
w = arb.World()

Hprev = np.dot(np.eye(4), htr.transl([-4,0,0]))
Hnext = np.dot(np.eye(4), htr.transl([4,0,0]))

#creation du robot
for i in range(1,3):
    prev_body = w.bodies[i-1]
    left_frame = prev_body.newframe(Hnext,(prev_body.name +'-next'))
    
    new_body = arb.Body(name=('body%02d' % i))
    right_frame = new_body.newframe(Hprev,(new_body.name +'-prev'))

    joint = arb.HingeJoint(gpos = .5)
    
    w.addjoint(joint, left_frame, right_frame, name = ('joint%02d' % (i-1)))

#mise à jour de la géométrie du robot
w.geometric()

#animation
if True:
    if 0:
        import visu_mayavi as vmaya
        vw = vmaya.World(w)
    else:
        import visu_vpython as vpy
        vw = vpy.World(w)
        
    if 1:
        for t in range(100):
            print t
            w.joints[1].gpos=t/20.
            w.geometric()
            vw.update()

