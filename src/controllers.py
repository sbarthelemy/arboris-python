from abc import ABCMeta, abstractmethod
from numpy import array, zeros, ones, eye, dot

class JointController(object):
    __metaclass__ = ABCMeta

    def ndof(self):
        return len(self._dof_indices)


    @abstractmethod
    def update(self, dt):
        pass


    @abstractmethod
    def viscosity(self):
        pass


    @abstractmethod
    def torque(self):
        pass


class ProportionalDerivativeController(JointController):

    
    def torque(self):
        return ones(self.ndof())


    def viscosity(self):
        return zeros( (self.ndof(), self.ndof()) )

    def update(self, dt):
        pass
