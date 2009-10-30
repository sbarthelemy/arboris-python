#coding=utf-8
"""A set of WorldObservers.
"""
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from core import WorldObserver
from abc import ABCMeta, abstractmethod, abstractproperty
from numpy import dot, array, eye, linalg, vstack, hstack, zeros, diag
from time import time as _time
from massmatrix import principalframe

import logging
logging.basicConfig(level=logging.DEBUG)


class EnergyMonitor(WorldObserver):
    """Compute and store the world energy at each time step.
    
    **Example:**

        >>> from arboris.core import ObservableWorld, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = ObservableWorld()
        >>> obs = EnergyMonitor(w)
        >>> w.observers.append(obs)
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2])
        >>> #obs.plot()

    """
    
    def __init__(self, world):
        self._world = world

    def init(self):
        self.time = []
        self.kinetic_energy = []
        self.potential_energy = []
        self.mechanichal_energy = []
        self._com_pos = {}
        self._bodies = self._world.ground.iter_descendant_bodies
        for body in self._bodies():
            self._com_pos[body]  = principalframe(body.mass)[:,3]

    def update(self, dt):
        self.time.append(self._world.current_time)
        Ec = dot(self._world.gvel, 
                 dot(self._world.mass, self._world.gvel) )/2.
        self.kinetic_energy.append(Ec)
        Ep = 0.
        for body in self._bodies():
            h = dot( dot(body.pose, self._com_pos[body])[0:3], self._world.up)
            Ep += body.mass[3,3] * h
        Ep *= 9.81
        self.potential_energy.append(Ep)
        self.mechanichal_energy.append(Ec+Ep)

    def plot(self):
        """Plot the energy evolution.
        """
        from pylab import plot, show, legend, xlabel, ylabel, title
        plot(self.time, self.kinetic_energy)
        plot(self.time, self.potential_energy)
        plot(self.time, self.mechanichal_energy)
        legend(('kinetic','potential','mechanical'))
        title('Energy evolution')
        xlabel('time (s)')
        ylabel('energy (J)')
        show()


class PerfMonitor(WorldObserver):
    """

    **Example:**

        >>> from arboris.core import ObservableWorld, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = ObservableWorld()
        >>> obs = PerfMonitor(w)
        >>> w.observers.append(obs)
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2])
        >>> print obs.get_summary() #doctest: +ELLIPSIS
        total computation time (s): ...
        min computation time (s): ...
        mean computation time (s): ...
        max computation time (s): ...
        >>> #obs.plot()

    """
    def __init__(self, world, log = False):
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._last_time = None
        self._computation_time = []
        self._world = world

    def init(self):
        self._last_time = _time()

    def update(self, dt):
        current_time = _time()
        self._computation_time.append(current_time - self._last_time)
        self._last_time = current_time
        if self._logger is not None:
            self._logger.info('current time (s): %.3f', 
                              self._world.current_time)

    def plot(self):
        from pylab import plot, show, xlabel, ylabel, title
        plot(self._computation_time)
        title('Computation time for each simulation time step')
        xlabel('simulation time step')
        ylabel('computation time (s)')
        show()

    def get_summary(self):
        total = sum(self._computation_time)
        return """total computation time (s): {0}
min computation time (s): {1}
mean computation time (s): {2}
max computation time (s): {3}""".format(
    total, 
    min(self._computation_time), 
    total/len(self._computation_time), 
    max(self._computation_time))


class Hdf5Logger(WorldObserver):
    """An observer that logs what we need and saves it in an hdf5 file.
    """
    def __init__(self, world, save_dynamical_model = True):
        self._world = world
        self._save_dynamical_model = save_dynamical_model
        self._save_arborisViewer_data = True
    
    
    def init(self):
        self.timeline = []
        
        if self._save_arborisViewer_data:
            self.wbodies = self._world.getbodies()
            self.bodies = {}
            for b in self.wbodies:
                self.bodies[b] = []
            self.wpoints = [] #TODO: how to extract points in the simulation
            self.points = {}
            self.wwrenches = [] #TODO: how to get wrenches in the simulation
            self.wrenches = {}
            
        if self._save_dynamical_model:
            self.gpos = []
            self.gvel = []
            self.mass = []
            self.nleffects = []
    
    
    def update(self, dt):
        self.timeline.append(self._world._current_time)
        
        if self._save_arborisViewer_data:
            for b in self.wbodies:
                self.bodies[b].append(b.pose)
            for p in self.wpoints:
                pass #TODO: self._bodies[p].append(p.position)
            for w in self.wwrenches:
                pass #TODO: self._bodies[w].append(w.position)
                
        if self._save_dynamical_model:
            self.gpos.append(self._world.ground.childrenjoints[0].frames[1].pose) #TODO: maybe chage this
            self.gvel.append(self._world.gvel)
            self.mass.append(self._world.mass.copy())
            self.nleffects.append(self._world.nleffects.copy())
        
        
    def write_file(self, filename, dest_in_file = "xp", file_access = 'a'):
        import h5py
        f = h5py.File(filename, file_access)
        group = self._get_group(f, dest_in_file)
        group.attrs["timeline"] = self.timeline
        if self._save_arborisViewer_data:
            for b in self.wbodies:
                dset = group.create_dataset(b.name, data=array(self.bodies[b]))
                dset.attrs["arborisViewerType"] = "matrix"
                dset.attrs["arborisViewerParent"] = "ground"
            for p in self.wpoints:
                dset = group.create_dataset(p.name, data=array(self.points[p]))
                dset.attrs["arborisViewerType"] = "translate"
                dset.attrs["arborisViewerParent"] = "ground"
            for w in self.wwrenches:
                print "WARNING: CONSTRAINT has to be update!!!!!"
                dset = group.create_dataset(w.name, data=array(self.wrenches[w]))
                dset.attrs["arborisViewerType"] = "wrench"
                dset.attrs["arborisViewerParent"] = c.name #????
        
        if self._save_dynamical_model:
            f.create_dataset('gpos', data=self.gpos)
            f.create_dataset('gvel', data=self.gvel)
            f.create_dataset('mass', data=self.mass)
            f.create_dataset('nleffects', data=self.nleffects)
        f.close()
    
    def _get_group(self, f, dest): #TODO: maybe this code could be optimize
        inter_groups = []
        _dest = dest
        if _dest[0] is not '/':
            _dest = '/' + _dest
        if _dest[-1] is not '/':
            _dest =  _dest + '/'
        slash_list = []
        for i in range(len(_dest)):
            if _dest[i] is '/':
                slash_list.append(i)
        for i in range(len(slash_list)-1):
            inter_groups.append(_dest[(slash_list[i]+1):(slash_list[i+1])])
        
        group = f
        for ig in inter_groups:
            group = group.require_group(ig)
        return group

