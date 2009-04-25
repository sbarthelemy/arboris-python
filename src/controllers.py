from abc import ABCMeta, abstractmethod
from numpy import array, zeros, ones, eye, dot
from misc import NamedObject

class Controller(NamedObject):
    def __init__(self, name=None):
        NamedObject.__init__(self, name)

class BodyController(Controller):
    pass

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
    def torque(self):
        pass


class WeightController(BodyController):
    pass

class ProportionalDerivativeController(JointController):

    def __init__(self, name=None):
        JointController.__init__(self, name)

    def torque(self):
        return ones(self.ndof())


    def viscosity(self):
        return zeros( (self.ndof(), self.ndof()) )

    def update(self, dt):
        pass
