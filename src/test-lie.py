H1 = htr.rotx(45.0/180*3.14)
H1i = htr.inv(H1)
Ad1 = htr.adjoint(H1)
Ad1i = htr.adjoint(H1i)
T2 = rm.Transform()
H2 = T2.gethomogeneousmatrix()
h1 = rm.HomogeneousMatrix(H1)
t1 = rm.Twist([0, 0, 1, 0, 0, 0])


