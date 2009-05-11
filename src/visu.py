# coding=utf-8
"""
Visualization of a simulation
"""
import numpy as np
from abc import ABCMeta, abstractmethod

class Color(object):
    """ A class in order to have unified colors
    """
    __metaclass__ = ABCMeta
    
    def __init__(self, var1, var2=1.):
        self.rgba = (0., 0., 0., 1.)
        if type(var1) is str:
            if   var1 is 'white':
                self.rgba = [1.,1.,1.,var2]
            elif var1 is 'black':
                self.rgba = [0.,0.,0.,var2]
            elif var1 is 'red':
                self.rgba = [1.,0.,0.,var2]
            elif var1 is 'green':
                self.rgba = [0.,1.,0.,var2]
            elif var1 is 'blue':
                self.rgba = [0.,0.,1.,var2]
            elif var1 is 'yellow':
                self.rgba = [1.,1.,0.,var2]
            elif var1 is 'velvet':
                self.rgba = [1.,1.,0.,var2]
            elif var1 is 'cyan':
                self.rgba = [1.,1.,0.,var2]
            elif var1 is 'brown':
                self.rgba = [.5,0.,0.,var2]
        if type(var1) is list:
            if len(var1) is 4:
                self.rgba = var1
            elif len(var1) is 3:
                self.rgba = [var1[0], var1[1], var1[2], var2]
        if type(var1) is tuple:
            if len(var1) is 4:
                self.rgba = [var1[0], var1[1], var1[2], var1[3]]
            elif len(var1) is 3:
                self.rgba = [var1[0], var1[1], var1[2], var2]
    
    @abstractmethod
    def get(self):
        pass
    
    
    
    
class World(object):
    """ A drawable version of arboris.World
    """
    __metaclass__ = ABCMeta
    
    def __init__(self, world, scale=1.):
        self._world = world
        self._scale = scale
        self.bodies = []
        self.wrenches = []
    
    def update(self):
        pass
    
    @abstractmethod
    def add_body(self):
        pass
    

class Body(object):
    """ A drawable version of rigidmotion.Body
    """
    __metaclass__ = ABCMeta
    
    def __init__(self, body, scale=1., color=None):
        self._body = body
        self.frames = []
        self.links = []
        self.shapes = []
        self._color = color
        self._scale = scale
    
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
