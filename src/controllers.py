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

    def __init__(self, name=None):
        JointController.__init__(self, name)


    def update(self, dt):
        pass


    def gforce(self):
        return ones(self.ndof())


    def viscosity(self):
        return zeros( (self.ndof(), self.ndof()) )


