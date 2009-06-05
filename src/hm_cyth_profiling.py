#import arboris
import homogeneousmatrix_c as hm_c

import numpy as np
#resultat dans h_c_statfile1
for i in range(100000):
    hm_c.rotzyx(np.array([3.14/6, 3.14/4, 3.14/3]))

#resultat dans h_c_statfile2
#for i in range(100000):
#    hm_c.rotzyx1000(np.array([3.14/6, 3.14/4, 3.14/3]))


#import twistvector_c as t1
#import numpy as np

#tw=np.array([2.5, 3., 0.25, 1.33, 2.11, 3.5])
#for i in range(100000):
#    t1.exp(tw)
#    t1.adjacency(tw)
#for i in range(100000):
#    hm_c.transl(np.array([1., 1., 1.]))
