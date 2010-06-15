# coding=utf-8
"""A controller using QP

TODO: This module is still work in progress.
"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")

from arboris.core import World, Controller, Frame
from cvxmod import param, optvar, matrix, norm2, problem, minimize
from numpy import array, zeros, dot, sqrt

class Task(object):

    def __init__(self, controlled_frame, target_frame, world):
        assert isinstance(controlled_frame, Frame)
        assert isinstance(target_frame, Frame)
        self._controlled_frame = controlled_frame
        self._target_frame = target_frame
        self._world = world #TODO: remove

    def cost(self, dgvel):
        """Returns the cost function of the task.

        TODO: add a (possibly singular) weighting matrix (thus allow to control the orientation)

        """
        J_ = self._controlled_frame.jacobian[3:6,:]
        J = param(value=matrix(J_))
        dJ = self._controlled_frame.djacobian[3:6,:]
        gvel = self._world.gvel
        Pdes = self._target_frame.pose[0:3,3]
        cf = self._controlled_frame
        dVdes = 10.*dot(cf.pose[0:3,0:3].T, Pdes - cf.pose[0:3,3]) -\
            2.*sqrt(10.)*dot(J_, self._world.gvel)
        return norm2(J*dgvel + param(value=matrix(dot(dJ, gvel) - dVdes)))


class BalanceController(Controller):

    def __init__(self, world, tasks=[], name=None):
        assert isinstance(world, World)
        Controller.__init__(self, name=name)
        self.world = world
        self.frames = self.world.getframes()
        self._rec_tau = [] #TODO: move this elsewhere
        self._tasks = tasks

    def init(self, world):
        self._wndof = world.ndof

    def update(self, dt=None):
        """
        """
        self.initialize_LQP()

        self.get_situation()

        self.compute_objectives()

        self.write_tasks()

        self.write_constraints()

        self.solve_LQP()

        M = param('M', value=matrix(self.world.mass))
        N = param('N', value=matrix(self.world.nleffects))
        # variables:
        dgvel = optvar('dgvel', self._wndof)
        tau   = optvar('tau', self._wndof)
        #fc    = optvar('tau', self._wndof)
        gvel = param('gvel', value=matrix(self.world.gvel))
        taumax = param('taumax', value=matrix(array([10.,10.,10.])))

        ### resolution ###
        cost = norm2(tau)
        for task in self._tasks:
            cost += 100. * task.cost(dgvel)

        p = problem(minimize(cost))
        p.constr.append(M*dgvel + N*gvel == tau)
        p.constr.append(-taumax <= tau)
        p.constr.append( tau <= taumax)
        p.solve(True)
        tau = array(tau.value).reshape(self._wndof)
        self._rec_tau.append(tau)
        gforce = tau
        impedance = zeros( (self._wndof, self._wndof) )
        return (gforce, impedance)

    def initialize_LQP(self):
        self._Fopti = 0.
        self._p = problem(minimize(self._Fopti))

    def get_situation(self):
        pass

    def compute_objectives(self):
        pass

    def write_tasks(self):
        pass

    def write_constraints(self):
        pass

    def solve_LQP(self):
        pass


    def plot(self):
        from pylab import plot, show, legend, xlabel, ylabel, title
        plot(self._rec_tau)
        legend(('tau1','tau2','tau3'))
        title('Torque evolution')
        xlabel('time (s)')
        ylabel('torques (Nm)')
        show()
