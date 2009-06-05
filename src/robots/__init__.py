from simplearm import simplearm
from ball import ball, box, cylinder
from human36 import human36 as _human36

def human36(*pargs, **kargs):
    return  _human36(*pargs, **kargs)[0]

