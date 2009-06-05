# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

import os
import shutil
import sys
import glob
import doctest
import inspect

def _from_module(module, object):
    """
    Return true if the given object is defined in the given
    module.
    """
    if module is None:
        return True
    elif inspect.getmodule(object) is not None:
        return module is inspect.getmodule(object)
    elif inspect.isfunction(object):
        return module.__dict__ is object.func_globals
    elif inspect.isclass(object):
        return module.__name__ == object.__module__
    elif hasattr(object, '__module__'):
        return module.__name__ == object.__module__
    elif isinstance(object, property):
        return True # [XX] no way not be sure.
    else:
        raise ValueError("object must be a class or function")

def fix_module_doctests(module):
    module.__test__ = {}
    for name in dir(module):
       value = getattr(module, name)
       if inspect.isbuiltin(value) and \
          isinstance(value.__doc__, str) and \
          _from_module(module, value):
           module.__test__[name] = value.__doc__

def find_python_modules():
    modules = []
    for f in glob.glob('*.py'):
        modules.append(f[:-3])
    modules.remove('hm_cyth_profiling')
    modules.remove('stat_hmpython_profiling')
    modules.remove('stat_hmcython_profiling')
    modules.remove('test')
    return modules

def find_cython_modules():
    modules = []
    for f in glob.glob('*.pyx'):
        modules.append(f[:-4])
    return modules

def rundoctests(verbose=False):

    modules = find_python_modules()
    for name in modules:
        exec('import {0} as module'.format(name))
        doctest.testmod(module, verbose=verbose)

    modules = ('homogeneousmatrix',)
    for name in modules:
        exec('import {0} as module'.format(name))
        fix_module_doctests(module)
        doctest.testmod(module, verbose=verbose)

    doctest.testfile('../test/update_dynamic.rst')

if __name__ == "__main__":
    rundoctests(sys.argv[1:])


 
