#!python
import rigidmotion as rm
import numpy as np
import homogeneousmatrix as htr
import adjointmatrix as Ad
import controlwindow as cw
from visu import *

w = rm.World()


H0 = np.dot(np.eye(4), htr.transl([-2,0,0]))

for i in range(1,5):
    name = 'body%02d' % i
    b = rm.Body(name)
    b.addframe(H0,'frame01.01')

    j1 = rm.HingeJoint(
        name = ('joint%02d' % (i-1)),
        leftframe = w.bodies[i-1].frames[0],
        rightframe = b.frames[1],
        gpos = 0.707)
    w.addjoint(j1)


H1 = htr.rotx(45.0/180*3.14)
H1i = htr.inv(H1)
Ad1 = htr.adjoint(H1)
Ad1i = htr.adjoint(H1i)
T2 = rm.Transform()
H2 = T2.gethomogeneousmatrix()
h1 = rm.HomogeneousMatrix(H1)
t1 = rm.Twist([0, 0, 1, 0, 0, 0])

wv = WorldVisualizer(w)
wv.draw()

c = cw.controlwindow(wv)
