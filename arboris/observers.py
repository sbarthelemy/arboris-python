#coding=utf-8
"""A set of WorldObservers.
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
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

    def register(self, obj):
        pass

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

    def finish(self):
        pass

    def plot(self):
        """Plot the energy evolution.
        """
        from pylab import plot, show, legend, xlabel, ylabel, title, figure
        plot(self.times, self.kinetic_energy)
        plot(self.times, self.potential_energy)
        plot(self.times, self.mechanichal_energy)
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
        total sim time : ... s
        min step time  : ... s
        mean step time : ... s
        max step time  : ... s
        >>> #obs.plot()

    """
    def __init__(self, world, log = False):
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._last_time = None
        self._spent_time = []
        self._world = world

    def register(self, obj):
        pass

    def init(self):
        self.last_time = _time()
    
    def update(self, dt):
        self._spent_time.append(_time()-self.last_time)
        if self._logger is not None:
            self._logger.info('current time: %6.2f %%', 
                              self._world.current_time)
    
    def finish(self):
        pass

    def plot(self):
        from pylab import plot, show, xlabel, ylabel, title
        plot(self._spent_time)
        title('Computation times')
        xlabel('time step')
        ylabel('computation time (s)')
        show()

    def get_summary(self):
        total = sum(self._spent_time)
        return """total sim time : {0} s
min step time  : {1} s
mean step time : {2} s
max step time  : {3} s""".format(
    total, min(self._spent_time), total/len(self._spent_time), 
    max(self._spent_time)) 
