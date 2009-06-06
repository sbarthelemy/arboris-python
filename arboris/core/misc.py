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

