#import arboris
import homogeneousmatrix as hm

for i in range(100000):
    hm.rotzyx(3.14/6, 3.14/4, 3.14/3)

#import twistvector as t2
#import numpy as np
#tw=np.array([2.5, 3., 0.25, 1.33, 2.11, 3.5])
#for i in range(100000):
#    t2.exp(tw)
#    t2.adjacency(tw)
#for i in range(100000):
#    hm.transl([1, 1, 1])
