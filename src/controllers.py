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

    def __init__(self, joints, Kp=0., Kd=0., ref_gpos=0., ref_gvel=0., name=None):
            JointController.__init__(self, name)
            self.joint = joints
            self.Kp = Kp
            self.Kd = Kd
            self.ref_gpos = ref_gpos
            self.ref_gvel = ref_gvel
            
    def update(self, dt):
        pass

    
    def viscosity(self):
        return zeros( (self.ndof(), self.ndof()) )


    def gforce(self):
        return self.Kp*(self.ref_gpos-self.joint.gpos) + self.Kd*(self.ref_gvel-self.joint.gvel)


if __name__ == "__main__":
    
    from worldfactory import triplehinge
    w = triplehinge()
    Kp = 10.
    Kd = 2.5
    c0 = ProportionalDerivativeController(w.joints[0], Kp, Kd, 1.)
    c1 = ProportionalDerivativeController(w.joints[1], Kp, Kd, 2.)
    c2 = ProportionalDerivativeController(w.joints[2], Kp, Kd, 3.)
    w.add_jointcontroller(c0, w.joints[0:1])
    w.add_jointcontroller(c1, w.joints[1:2])
    w.add_jointcontroller(c2, w.joints[2:3])
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
        print w.joints[0].gpos[0]
        vw.viewer.frame()
