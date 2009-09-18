# coding=utf-8
"""This module is still work in progress.
"""
__author__ = ("Joseph Salini <joseph.salini@gmail.com>")

from arboris.core import World, Controller
from arboris.homogeneousmatrix import iadjoint, transl
from cvxmod import param, optvar, matrix, norm2, problem, minimize
from numpy import array, zeros, dot, sqrt

class BalanceController(Controller):

    def __init__(self, world, name=None):
        assert isinstance(world, World)
        Controller.__init__(self, name=name)
        self.world = world
        self.frames = self.world.getframesdict()
        self._rec_tau = [] #TODO: move this elsewhere
        
    def _opt_params(self, Pdes, frame):
        """Return the cvxmod parameters corresponding to the dynamical model.
        """
    
        # vars with a _ suffix are numerical
        M = param('M', value=matrix(self.world.mass))
        N = param('N', value=matrix(self.world.nleffects))
        J_ = dot(iadjoint(transl(0,0,0)), frame.jacobian)[3:6,:]
        J = param('J', value=matrix(J_))
        dJ = param('dJ', value=matrix(dot(iadjoint(transl(0,0,0)), 
                                          frame.djacobian)[3:6,:]))
        dTdes_ = 10.*dot(frame.pose[0:3,0:3].T, 
                         Pdes - frame.pose[0:3,3]) -\
                         2.*sqrt(10.)*dot(J_, self.world.gvel)
        dTdes = param('dTdes', value=matrix(dTdes_))
        return (M, N, J, dJ, dTdes)
    
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

        # task frame:
        f = self.frames['EndEffector']      
        Pdes = self.frames['Target'].pose[0:3,3]
        
        # params:
        (M, N, J, dJ, dTdes) = self._opt_params(Pdes, f)
        
        # variables:
        dgvel = optvar('dgvel', self._wndof)
        tau   = optvar('tau', self._wndof)
        #fc    = optvar('tau', self._wndof)        
        gvel = param('gvel', value=matrix(self.world.gvel))
        taumax = param('taumax', value=matrix(array([10.,10.,10.])))
        
        ### resolution ###
        Fopti = norm2(tau) + 100.*norm2(J*dgvel + dJ*gvel - dTdes)
        p = problem(minimize(Fopti))
        p.constr.append( M*dgvel + N*gvel == tau )
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
