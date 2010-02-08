# coding=utf-8
__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from abc import ABCMeta, abstractmethod, abstractproperty
from core import NamedObject, Controller, World
from numpy import array, zeros, dot, ix_
from numpy.linalg import norm
import homogeneousmatrix
from joints import LinearConfigurationSpaceJoint

class WeightController(Controller):
    """A contoller which applies weight to joints.

    **Test:**: 
    >>> from arboris.core import simplearm
    >>> w = simplearm()
    >>> joints = w.getjoints()
    >>> joints['Shoulder'].gpos[0] = 3.14/4
    >>> joints['Elbow'].gpos[0] = 3.14/4
    >>> joints['Wrist'].gpos[0] = 3.14/4
    >>> c = WeightController()
    >>> w.register(c)
    >>> w.init()
    >>> w.update_dynamic() #TODO change for update_kinematic
    >>> (gforce, impedance) = c.update() #TODO: test!
    
    """
    def __init__(self, gravity=-9.81, name=None):
        self.gravity = float(gravity)
        Controller.__init__(self, name=name)


    def init(self, world):
        assert isinstance(world, World)
        self._bodies = filter(lambda x: norm(x.mass>0.),
                world.ground.iter_descendant_bodies())
        self._wndof = world.ndof
        self._gravity_dtwist = zeros(6)
        self._gravity_dtwist[3:6] = float(self.gravity)*world.up

    def update(self, dt=None):
        gforce = zeros(self._wndof)
        for b in self._bodies:
            #TODO: improve efficiency
            from arboris.massmatrix import principalframe
            H_bc = principalframe(b.mass)
            R_gb = b.pose[0:3,0:3] 
            H_bc[0:3,0:3] = R_gb.T
            #wrench_c = array((0,0,0,0,-9.81*b.mass[3,3],0))
            #Ad_cb = homogeneousmatrix.iadjoint(H_bc)
            #wrench_b = dot(Ad_cb.T, wrench_c)
            #gforce += dot(b.jacobian.T, wrench_b )
            # gravity acceleration expressed in body frame
            g = dot(homogeneousmatrix.iadjoint(b.pose), self._gravity_dtwist)
            gforce += dot(b.jacobian.T, dot(b.mass, g))
        impedance = zeros( (self._wndof, self._wndof) )
        return (gforce, impedance)
            

class ProportionalDerivativeController(Controller):
    r"""A proportional-derivative controller.

    A 1-dimensional PD controller would deliver a torque `\tau`:

    .. math::
        \tau(t) &= K_p (q_d - q(t+dt)) + K_d (\dot{q}_d - \dot{q}(t+dt))

    where `q_d` and `\dot{q}_d` are the respectively the desired position 
    and velocity and where `\tau(t)` is assumed constant on the `[t,t+dt]`
    interval.

    Here

    .. math::
        q(t+dt) = q(t) + dt \dot{q}(t+dt)

    so

    .. math::
        \tau(t) &= K_p (q_d - (q(t) + dt \dot{q}(t+dt))) 
            + K_d (\dot{q}_d - \dot{q}(t+dt)) \\
                &= K_p (q_d-q(t)) + K_d \dot{q}_d 
            - (K_p dt + K_d)\dot{q}(t+dt) \\
                &=  \tau_0(t) + Z(t) \dot{q}(t+dt)

    with

    .. math::
        \tau_0(t) &=  K_p (q_d-q(t)) + K_d \dot{q}_d \\
        Z(t) &= -(K_p dt + K_d)

    This result can be easily generalized to the `n`-dimensional case.

    """
    def __init__(self, joints, kp=None, kd=None, gpos_des=None, 
                 gvel_des=None, name=None):
        Controller.__init__(self, name=name)
        self._cndof = 0
        dof_map = []
        for j in joints:
            if not isinstance(j, LinearConfigurationSpaceJoint):
                raise ValueError('Joints must be LinearConfigurationSpaceJoint instances')
            else:
                self._cndof += j.ndof
                dof_map.extend(range(j.dof.start, j.dof.stop))
        self._dof_map = array(dof_map)
        self.joints = joints

        if kp is None:
            self.kp = zeros((self._cndof, self._cndof))
        else :
            self.kp = array(kp).reshape((self._cndof, self._cndof))

        if kd is None:
            self.kd = zeros((self._cndof, self._cndof))
        else :
            self.kd = array(kd).reshape((self._cndof, self._cndof))

        if gpos_des is None:
            self.gpos_des = zeros(self._cndof)
        else:
            self.gpos_des = array(gpos_des).reshape(self._cndof)

        if gvel_des is None:
            self.gvel_des = zeros(self._cndof)
        else:
            self.gvel_des = array(gvel_des).reshape(self._cndof)
    
    def init(self, world):
        self._wndof = world.ndof
        dof_map = []
        for j in self.joints:
                dof_map.extend(range(j.dof.start, j.dof.stop))
        self._dof_map = array(dof_map)

    def update(self, dt):
        """
        """
        gforce = zeros(self._wndof)
        impedance = zeros((self._wndof, self._wndof))
        
        gpos = [] 
        gvel = []
        for j in self.joints:
            gpos.append(j.gpos)
            gvel.append(j.gvel)
        gpos = array(gpos).reshape(self._cndof)
        gvel = array(gvel).reshape(self._cndof)
        gforce[self._dof_map] = \
                dot(self.kp, self.gpos_des - gpos) + \
                dot(self.kd, self.gvel_des)
        impedance[ix_(self._dof_map, self._dof_map)] = -(dt*self.kp+self.kd)
        return (gforce, impedance)


