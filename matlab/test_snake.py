# coding=utf-8
"""
This script runs the same simulation from matlab and python and compares the results.

The "snake" robot from arboris-python is converted to arboris-matlab.
"""
__author__ = ("Sébastien Barthélemy <sebastien.barthelemy@gmail.com>")

from arboris.robots.snake import add_snake
from numpy import array, arange, dot
from simulation import *

def kenergy(xp):
    kinetic_energy = []
    for (gvel, M) in zip(xp['gvel'].value, xp['mass'].value):
        kinetic_energy.append(dot(gvel, dot(M, gvel) )/2.)
    return kinetic_energy

def plot_energy(xp_mat, xp_py, ymax=None):
    """Compute and plot the energy
    """
    time = xp_mat['time'].value
    en_mat = kenergy(xp_mat)
    en_py = kenergy(xp_py)
    from pylab import plot, show, legend, xlabel, ylabel, title, axis
    plot(time, en_mat)
    plot(time, en_py)
    #plot(time, [m-p for (m,p) in zip(en_mat, en_py)])
    legend(('kinetic matlab','kinetic python'))
    #legend(('kinetic matlab','kinetic python','kinetic matlab - python'))
    title('Energy evolution')
    xlabel('time (s)')
    ylabel('energy (J)')
    if ymax is not None:
        ax = list(axis())
        ax[3] = ymax 
        axis(ax)
    show()
        
t_start, t_end, dt = 0., 2.08, 0.005
t_end = 1.430 ####
timeline = arange(t_start, t_end, dt)
with_weight = False
is_fixed = False

world = ObservableWorld()
world._up = array([0., 0., 1.]) # we use matlab's convention
njoints = 9
lengths = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
masses = [1., .9, .8 , .7 , .6, .5, .4 , .3, .2]
gvel = gvel=[2.]*njoints
gpos = [0, 3.14159/4., 0, 0, 0, 0, 0, 0, 0]
add_snake(world, njoints, lengths, masses, gvel=[2.]*njoints, is_fixed=is_fixed)

name = "haha"

if with_weight:
    world.register(WeightController(world))
    
simulate_mat(world, timeline, name)
simulate_py(world, timeline, name)

matpy = load_matpy(name)
d = diff_matpy(*matpy)
print max_error(d)
plot_energy(*matpy)

