# coding=utf-8
"""
This module run simulation with arboris-python and arboris-matlab, store 
the result in hdf5 files and compare them.

"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")

import os
import time
import h5py
from pylab import plot, show, xlabel, ylabel, title, figure, legend
from arboris.core import World, ObservableWorld, simulate, SubFrame, WorldObserver
from numpy import array, arange, linalg

class RobotObserver(WorldObserver):
    """An observer that will log what we need and save it in an hdf5 file.
    """
    def __init__(self, world):
        self._world = world
    
    def init(self):
        self._joints = self._world.getjoints()
        self._root = self._world.getbodies()[1]
        self.time = []
        self.H0 = []
        self.q = []
        self.dq = []
        self.M = []
        self.N = []
    
    def update(self, dt):
        print self._world.current_time
        q=[]
        for j in self._joints:
            q.append(j.gpos)
        self.time.append(self._world.current_time)
        self.H0.append(self._root.pose)
        self.q.append(q)
        self.dq.append(self._world.gvel)
        self.M.append(self._world.mass.copy())
        self.N.append(self._world.nleffects.copy())
        
    def save_data(self, filename):
        import h5py
        f = h5py.File(filename, 'w')
        f.create_dataset('time', data=self.time)
        f.create_dataset('H0', data=self.H0)
        #f.create_dataset('q', data=self.q)
        f.create_dataset('dq', data=self.dq)
        f.create_dataset('N', data=self.N)
        f.create_dataset('M', data=self.M)
        f.close()



def run_py(nb_bd, w0, v0, dq0, with_gravity, dt, tend, name):
    """Run an arboris-python simulation."""
    gvel = []
    gvel.extend(w0)
    gvel.extend(v0)
    gvel.extend(dq0)
    if with_gravity:
        gravity = -9.81
    else:
        gravity = 0.
    
    world = ObservableWorld()
    world._up = array([0., 0., 1.])
    import robot_generic
    robot_generic.add_robot(world)
    obs = RobotObserver(world)
    world.observers.append(obs)
    from arboris.controllers import WeightController
    world.register( WeightController(world, gravity) )
    world.init()
    
    joints = world.getjoints()
    joints[0].gvel = gvel[0:6]
    for i in range(len(joints)-1):
        joints[i+1].gvel = [gvel[6+i]]

    time = arange(0, tend, dt)
    simulate(world, time)
    file_name = name + '_py.h5' 
    obs.save_data(file_name)
    
def run_mat(nb_bd, w0, v0, dq0, with_gravity, dt, tend, name):
    """Run an arboris-matlab simulation."""
    gvel = []
    gvel.extend(v0)
    gvel.extend(w0)
    gvel.extend(dq0)
    if with_gravity:
        gravity = [0,0,-9.81,0,0,0]
    else:
        gravity = [0,0,0,0,0,0]

    cmd = """matlab -nojvm -r "simulate_matlab({nb_bd},{T0},{gravity},{dt},{tend},'{name}')" """.format(
        nb_bd=nb_bd, T0=gvel, gravity=gravity, dt=dt, tend=tend, name=name)
    print cmd
    #matlab_exe_time = time.time()
    os.system(cmd)
    #matlab_exe_time = time.time() - matlab_exe_time

def _troncate(data_dict, diff):
    for k,v in data_dict.iteritems():
        data_dict[k] = v[:diff]

def _transpose_sequence(Mlist):
    Mout = Mlist.copy()
    for i in range(len(Mlist)):
        Mout[i] = Mlist[i].T
    return Mout

def _inverse_root_values(M):
    """Convert a matrix from Matlab to python:
    2 things to change, the order of the 6 first rows and columns
    and to transpose every matrix because the matlab save a transposed matrix in the .h5 file
    """
    if len(M.shape) == 2:
        Mout = M.copy()
        Mout[:,0:3] = M[:,3:6].copy()
        Mout[:,3:6] = M[:,0:3].copy()
    else:
        Mint = M.copy()
        Mint[:,0:3,:] = M[:,3:6,:].copy()
        Mint[:,3:6,:] = M[:,0:3,:].copy()
        Mout = Mint.copy()
        Mout[:,:,0:3] = Mint[:,:,3:6].copy()
        Mout[:,:,3:6] = Mint[:,:,0:3].copy()
    return Mout
    
def extract_data(filename, is_matlab=False):
    """Get data from an hdf5 file and return it as a dict."""
    f = h5py.File(filename, 'r')
    time = f['time'].value
    H0 = f['H0'].value
    #q = f['q'].value
    dq = f['dq'].value
    M = f['M'].value
    N = f['N'].value
    f.close()
    data = {'time':time, 'H0':H0, 'dq':dq, 'M':M, 'N':N}
    if is_matlab:
        # do some conversion
        # TODO: do it before saving the hdf5 file!
        _troncate(data, -2)
        data['H0'] = _transpose_sequence(data['H0'])
        data['dq'] = _inverse_root_values(data['dq'])
        data['M']  = _inverse_root_values(_transpose_sequence(data['M']))
        data['N']  = _inverse_root_values(_transpose_sequence(data['N']))
    return data
        
def compute_diff(xp0, xp1):
    assert xp0['time'].shape[0] == xp1['time'].shape[0]
    diff = {}
    for k in xp0:
        diff[k] = xp1[k] - xp0[k]
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
    plot(time, diff['H0'][:,0:3,3])
    title('Root body position Difference')
    
    figure()
    plot(time, ewnorm(diff['H0'][:,0:3,0:3]))
    title('Root body rotational Difference')
    
    figure()
    plot(time, ewnorm(diff['dq']))
    title('Generalized Velocity Difference')
        
    figure()
    plot(time, ewnorm(diff['M']))
    title('Mass Matrix norm difference')
    
    figure()
    plot(time, ewnorm(diff['N']))
    title('Non-linear Effects Matrix norm difference')
    
    show()

def load_matpy(filename):
    matxp = extract_data(filename+'_mat.h5', True)
    pyxp = extract_data(filename+'_py.h5', False)
    return (matxp, pyxp)
    
def diff_matpy(filename):
    return compute_diff(*load_matpy(filename))

    
