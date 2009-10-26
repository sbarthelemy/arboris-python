

>>> from arboris.homogeneousmatrix import *
>>> from numpy.linalg import norm

>>> (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
>>> norm(rotzyx(az,ay,ax) - dot(rotz(az),dot(roty(ay),rotx(ax)))) < 1e-10
True


