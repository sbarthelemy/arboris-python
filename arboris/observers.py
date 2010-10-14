#coding=utf-8
"""A set of Observers.
"""
__author__ = (u"Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")
import arboris.core
from abc import ABCMeta, abstractmethod, abstractproperty
from numpy import dot, array, eye, linalg, vstack, hstack, zeros, diag
from time import time as _time
from arboris.massmatrix import principalframe
import logging
logging.basicConfig(level=logging.DEBUG)

class EnergyMonitor(arboris.core.Observer):
    """Compute and store the world energy at each time step.

    **Example:**

        >>> from arboris.core import World, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = World()
        >>> observers = [EnergyMonitor()]
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2], observers)
        >>> #obs.plot()

    """

    def init(self, world, timeline):
        self._world = world
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
        from pylab import plot, show, legend, xlabel, ylabel, title
        plot(self.time, self.kinetic_energy)
        plot(self.time, self.potential_energy)
        plot(self.time, self.mechanichal_energy)
        legend(('kinetic','potential','mechanical'))
        title('Energy evolution')
        xlabel('time (s)')
        ylabel('energy (J)')
        show()


class PerfMonitor(arboris.core.Observer):
    """

    **Example:**

        >>> from arboris.core import World, simulate
        >>> from arboris.robots.simplearm import add_simplearm
        >>> w = World()
        >>> obs = PerfMonitor()
        >>> add_simplearm(w)
        >>> simulate(w, [0,1,2], [obs])
        >>> print obs.get_summary() #doctest: +ELLIPSIS
        total computation time (s): ...
        min computation time (s): ...
        mean computation time (s): ...
        max computation time (s): ...
        >>> #obs.plot()

    """
    def __init__(self, log=False):
        arboris.core.Observer.__init__(self)
        if log:
            self._logger = logging.getLogger(self.__class__.__name__)
        else:
            self._logger = None
        self._last_time = None
        self._computation_time = []

    def init(self, world, timeline):
        self._world = world
        self._last_time = _time()

    def update(self, dt):
        current_time = _time()
        self._computation_time.append(current_time - self._last_time)
        self._last_time = current_time
        if self._logger is not None:
            self._logger.info('current time (s): %.3f',
                              self._world.current_time)

    def finish(self):
        pass

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


class Hdf5Logger(arboris.core.Observer):
    """An observer that saves the simulation data in an hdf5 file.

    :param filename: name of the output hdf5 file
    :type filename: str
    :param group: subgroup within the hdf5 file. Defaults to "/"
    :type group: str
    :param mode: mode used to open the hdf5 file. Can be 'w' or 'a' (default)
    :type mode: str
    :param save_state: toggle the write of the ``gpos`` and ``gvel`` groups
    :type save_state: bool
    :param save_transforms: toggle the write of the ``transforms`` group
    :type save_transforms: bool
    :param flat: whether to save body of joint poses in the ``transforms`` group
    :type flat: bool
    :param save_model: toggle the write of ``model`` group
    :type save_model: bool

    All the simulation data lies in a single user-chosen group, that is denoted
    ``root`` in the following, and which defaults to ``/``. All the data are in
    S.I. units (radians, meters, newtons and seconds).

    The hdf5 file has the following layout::

      root/timeline (nsteps,)
      root/gpositions/
      root/gvelocities/
      root/model/
      root/transforms/

    The ``gvelocities`` group contains the generalized velocities of the
    joints::

      NameOfJoint0 (nsteps, joint0.ndof)
      NameOfJoint1 (nsteps, joint1.ndof)
      ...

    while the ``gpositions`` group contains their generalized positions.

    The ``model`` group contains the matrices from the dynamical model::

      gvel (nsteps, ndof)
      gforce (nsteps, ndof)
      mass (nsteps, ndof, ndof)
      nleffects (nsteps, ndof, ndof)
      admittance (nsteps, ndof, ndof)

    The ``transforms`` group contains homogeneous transformations,
    useful for viewing an animation of the simulation.

      NameOfTransform0 (nsteps, 4, 4)
      NameOfTransform1 (nsteps, 4, 4)
      ...

    The name and value of the transforms depends on the ``flat`` parameter.
    If ``flat`` is True, then there is one transform per body, named after the
    body and whose value is the body absolute pose (``Body.pose``).
    If ``flat`` is False, there is one transform per joint, whose value is
    ``Joint.pose`` and whose name is taken from the joint second frame
    (``Joint.frames[1].name``).

    """
    def __init__(self, filename, group="/", mode='a', save_state=False,
                 save_transforms=True, flat=False, save_model=False):
        import h5py
        arboris.core.Observer.__init__(self)
        # hdf5 file handlers
        self._file = h5py.File(filename, mode)
        self._root = self._file
        for g in group.split('/'):
            if g:
                self._root = self._root.require_group(g)
        # what to save
        self._save_state = save_state
        self._save_transforms = save_transforms
        self._flat = flat
        self._save_model = save_model

    @property
    def root(self):
        return self._root

    def init(self, world, timeline):
        """Create the datasets.
        """
        self._world = world
        self._nb_steps = len(timeline)-1
        self._current_step = 0
        self._root.require_dataset("timeline", (self._nb_steps,), 'f8')
        if self._save_state:
            self._gpositions = self._root.require_group('gpositions')
            self._gvelocities = self._root.require_group('gvelocities')
            for j in self._world.getjoints():
                self._gpositions.require_dataset(j.name,
                        (self._nb_steps,) + j.gpos.shape[:])
                self._gvelocities.require_dataset(j.name,
                        (self._nb_steps, j.ndof))
        if self._save_transforms:
            self._arb_transforms = {}
            self._transforms = self._root.require_group('transforms')
            if self._flat:
                for b in self._world.iterbodies():
                    self._arb_transforms[b.name] = b
            else:
                for j in  self._world.getjoints():
                    self._arb_transforms[j.frames[1].name] = j
            for f in self._world.itermovingsubframes():
                self._arb_transforms[f.name] = f
            for k in self._arb_transforms.iterkeys():
                d = self._transforms.require_dataset(k,
                                                    (self._nb_steps, 4,4),
                                                    'f8')
        if self._save_model:
            self._model = self._root.require_group('transforms')
            ndof = self._world.ndof
            self._model.require_dataset("gvel",
                    (self._nb_steps, ndof), 'f8')
            self._model.require_dataset("mass",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("admittance",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("nleffects",
                    (self._nb_steps, ndof, ndof), 'f8')
            self._model.require_dataset("gforce",
                    (self._nb_steps, ndof), 'f8')

    def update(self, dt):
        """Save the current data (state...).
        """
	assert self._current_step <= self._nb_steps
        self._root["timeline"][self._current_step] = self._world._current_time
        if self._save_state:
            for j in self._world.getjoints():
                self._gpositions[j.name][self._current_step] = j.gpos
                self._gvelocities[j.name][self._current_step] = j.gvel
        if self._save_transforms:
            for k, v in self._arb_transforms.iteritems():
                if isinstance(v, arboris.core.MovingSubFrame):
                    if self._flat:
                        pose = v.pose
                    else:
                        pose = v.bpose
                else:
                    pose = v.pose
                self._transforms[k][self._current_step,:,:] = v.pose
        if self._save_model:
            self._model["gvel"][self._current_step,:] = self._world.gvel
            self._model["mass"][self._current_step,:,:] = self._world.mass
            self._model["admittance"][self._current_step,:,:] = \
                    self._world.admittance
            self._model["nleffects"][self._current_step,:,:] = \
                    self._world.nleffects
            self._model["gforce"][self._current_step,:] = self._world.gforce
        self._current_step += 1

    def finish(self):
        self._file.close()
