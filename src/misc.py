# coding=utf-8
"""
Miscaleneous classes

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")



class NamedObject(object):
    """
    A class for anything named to depend from.
    """

    def __init__(self, name=None):
        self.name = name

class DuplicateNameError(Exception):
    pass

class ColoredObject(object):
   pass

class Color(object):
    """ A class in order to have unified colors
    """
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
    
