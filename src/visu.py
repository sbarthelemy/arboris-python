# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
from abc import ABCMeta, abstractmethod


class World(object):
    """ A drawable version of arboris.World
    """
    __metaclass__ = ABCMeta
    
    def __init__(self, world):
        self._world = world
        self.bodies = []
        for b in self._world.bodies:
            self.add_body(b)
    
    def update(self):
        for b in self.bodies:
            b.update()
    
    @abstractmethod
    def add_body(self):
        pass
    

class Body(object):
    """ A drawable version of rigidmotion.Body
    """
    __metaclass__ = ABCMeta
    
    def __init__(self, body):
        self._body = body
        self.frames = []
        self.links = []
        self.draw_body()
    
    @abstractmethod
    def draw_body(self):
        pass
    
    #@abstractmethod    
    #def update(self):
    #    pass
        
    #@abstractmethod        
    #def draw_frame(self, pose=np.eye(4), label=None, parent=None, length=1):
    #    pass
        
    #@abstractmethod
    #def draw_link(self):
    #    pass
    
    #@abstractmethod
    #def draw_shape(self):
    #    pass
