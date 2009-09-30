# coding=utf-8
"""
This module runs 2 simulations from matlab and python and compares the results

"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")


from simulation import *
import time

# initialize velocity:
v0  = [0,0,0]
w0  = [0,0,1]

# miscellaneous parameters
with_gravity = 1 
dt = 0.001 # simulation time step
tend = .01 # time of simulation

error = {}
d = {}
mat = {}
py = {}
for nbd in [1]: # number of bodies in the simulation:
    for with_gravity in [0]: 
        dq0 = [0]*(nbd-1)
        name = "nbd{nbd}_w{w[0]}{w[1]}{w[2]}_v{v[0]}{v[1]}{v[2]}_g{g}".format(nbd=nbd, w=w0, v=v0, g=with_gravity)
        run_mat(nbd, w0, v0, dq0, with_gravity, dt, tend, name)
        run_py(nbd, w0, v0, dq0, with_gravity, dt, tend, name)
        (mat[name], py[name]) = load_matpy(name)
        diff = diff_matpy(name)
        error[name] = max_error(diff)
        d[name] = diff


