from abc import ABCMeta, abstractmethod
from numpy import array, zeros, ones, eye, dot
from misc import NamedObject
import homogeneousmatrix

class Controller(NamedObject):
    def __init__(self, name=None):
        NamedObject.__init__(self, name)

class JointController(Controller):
    __metaclass__ = ABCMeta

    def ndof(self):
        return len(self._dof)

            
    @abstractmethod
    def update(self, dt):
        pass


    @abstractmethod
    def viscosity(self):
        pass


    @abstractmethod
    def gforce(self):
        pass


class WeightController(JointController):

    def __init__(self, ground, gravity=None):

        self.ground = ground
        if gravity is None:
            self._gravity = array([0., 0., 0., 0., -9.81, 0.])
        else:
            self._gravity = gravity


    def update(self,dt):
        self._gforce = zeros(self.ndof())
        for b in self.ground.descendants():
            # gravity acceleration expressed in body frame
            g = dot(homogeneousmatrix.iadjoint(b.pose), self._gravity)
            self._gforce += dot(b.jacobian.T, dot(b.mass, g))
            

    def viscosity(self):
        return zeros( (self.ndof(), self.ndof()) )


    def gforce(self):
        return self._gforce




class ProportionalDerivativeController(JointController):

    def __init__(self, joints, Kp=None, Kd=None, gpos_des=None, 
                 gvel_des=None, name=None):
            ndof = 0
            from joints import LinearConfigurationSpaceJoint
            for j in joints:
                if not isinstance(j, LinearConfigurationSpaceJoint):
                    raise ValueError('Joints must be LinearConfigurationSpaceJoint instances')
                ndof += j.ndof()
            JointController.__init__(self, name)
            self.joints = joints
            if Kp is None:
                self.Kp = zeros((ndof, ndof))
            else:
                self.Kp = array(Kp)

            if Kd is None:
                self.Kd = zeros((ndof,ndof))
            else:
                self.Kd = array(Kd)

            if gpos_des is None:
                self.gpos_des = zeros(ndof)
            else:
                self.gpos_des = array(gpos_des)

            if gvel_des is None:
                self.gvel_des = zeros(ndof)
            else:
                self.gvel_des = array(gvel_des)
            
    def update(self, dt):
        pass

    
    def viscosity(self):
        """
        TODO: return non-zero viscosity
        """
        return zeros( (self.ndof(), self.ndof()) )


    def gforce(self):
        
        gpos = []
        gvel = []
        for j in self.joints:
            gpos.append(j.gpos)
            gvel.append(j.gvel)
        gpos = array(gpos).reshape(self.ndof())
        gvel = array(gvel).reshape(self.ndof())
        return dot(self.Kp, self.gpos_des - gpos) + \
               dot(self.Kd, self.gvel_des - gvel)

if __name__ == "__main__":
    
    from worldfactory import triplehinge
    from numpy import diag
    w = triplehinge()
    Kp = diag([10.,5.,1.],0)
    Kd = diag([2.5,1.,0.1],0)
    gpos_des = [1.,2.,3.]
    c0 = ProportionalDerivativeController(w.joints[0:3], Kp, Kd, gpos_des)
    w.add_jointcontroller(c0, w.joints[0:3])
    w.update_dynamic()
    
    import visu_osg
    vw = visu_osg.World_Factory(w, 0.1)
    vw.viewer.realize()
    dt = 0.001
    while(not(vw.viewer.done())):
    
        w.update_dynamic()
        w.update_controllers(dt)
        w.integrate(dt)
        vw.update(True, True)
        vw.viewer.frame()
