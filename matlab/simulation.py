# coding=utf-8
"""
This module provides helpers to run simulation with arboris-python and 
arboris-matlab, store the result in hdf5 files and compare them.

"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>", 
              "Sébastien Barthélemy <sebastien.barthelemy@gmail.com>")

import os
import time
import h5py
from pylab import plot, show, xlabel, ylabel, title, figure, legend
from arboris.core import World, simulate, SubFrame
from numpy import array, arange, linalg
from arboris.observers import Hdf5Logger
from convert_py2mat import MatlabSimulationGenerator

def simulate_mat(world, timeline, name):
    stream = open('simulate.m', 'w')
    MatlabSimulationGenerator().generate_simulation(world, timeline, stream, name)
    stream.close()
    cmd = 'matlab -nojvm -r "simulate()"'
    os.system(cmd)

def simulate_py(world, timeline, name):
    observers = [Hdf5Logger(name+'_py.h5')]
    simulate(world, timeline, observers)

def load_matpy(name, mode='r'):
    matxp = h5py.File(name+'_mat.h5', mode)
    pyxp = h5py.File(name+'_py.h5', mode)
    return (matxp, pyxp)
    
def diff_matpy(xp_mat, xp_py):
    return compute_diff(xp_mat, xp_py)

def compute_diff(xp0, xp1):
    assert xp0['time'].value.shape[0] == xp1['time'].value.shape[0]
    diff = {}
    for k in xp0:
        diff[k] = xp1[k].value - xp0[k].value
    return diff

def max_error(diff):
    from numpy import max, abs
    err = {}
    for k in diff:
        err[k] = max(abs(diff[k]))
    return err

def plot_diff(diff, time=None):
    from numpy.linalg import norm
    
    if time is None:
        time = range(len(diff['time']))
    else:
        assert len(time) == len(diff['time'])
    
    def ewnorm(iterable):
        return [norm(i) for i in iterable]
        
    figure()
    plot(time, diff['gpos'][:,0:3,3])
    title('Root body position Difference')
    
    figure()
    plot(time, ewnorm(diff['gpos'][:,0:3,0:3]))
    title('Root body rotational Difference')
    
    figure()
    plot(time, ewnorm(diff['gvel']))
    title('Generalized Velocity Difference')
        
    figure()
    plot(time, ewnorm(diff['mass']))
    title('Mass Matrix norm difference')
    
    figure()
    plot(time, ewnorm(diff['nleffects']))
    title('Non-linear Effects Matrix norm difference')
    
    show()



    
