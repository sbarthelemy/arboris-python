# coding=utf-8
"""
Miscaleneous classes

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")
import Exception 
#cimport Exception

cdef class NamedObject(object):
    """
    A class for anything named to depend from.
    """

    def __init__(self, name=None):
        self.name = name

def class DuplicateNameError(Exception):    #can't declare this one cdef 'cause Exception is not an extended object...
    pass                                    #must see an equivalent in cython

