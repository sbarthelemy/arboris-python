from abc import ABCMeta, abstractmethod, abstractproperty
from core import Controller
from numpy import array, zeros, dot, ix_
from misc import NamedObject
import homogeneousmatrix
from joints import LinearConfigurationSpaceJoint

class WeightController(Controller):

    def __init__(self, world, gravity=None, name=None):
        self.ground = world.ground
        self._wndof = world.ndof
        if gravity is None:
            self._gravity = array([0., 0., 0., 0., -9.81, 0.])
        else:
            self._gravity = gravity
        Controller.__init__(self, name=name)

    def update(self,dt):
        gforce = zeros(self._wndof)
        for b in self.ground.iterdescendants():
            # gravity acceleration expressed in body frame
            g = dot(homogeneousmatrix.iadjoint(b.pose), self._gravity)
            gforce += dot(b.jacobian.T, dot(b.mass, g))

        viscosity = zeros( (self._wndof, self._wndof) )
        return (gforce, viscosity)
            

class ProportionalDerivativeController(Controller):

    def __init__(self, wndof, joints, kp=None, kd=None, gpos_des=None, 
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
        self._wndof = wndof

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

    def update(self, dt, t):
        """
        TODO: return non-zero viscosity
        """
        gforce = zeros(self._wndof)
        viscosity = zeros((self._wndof, self._wndof))
        
        gpos = [] 
        gvel = []
        for j in self.joints:
            gpos.append(j.gpos)
            gvel.append(j.gvel)
        gpos = array(gpos)
        gvel = array(gvel)
        gforce[ix_(self._dof_map)] = \
                dot(self.kp, self.gpos_des - gpos) + \
                dot(self.kd, self.gvel_des - gvel)
        return (gforce, viscosity)


