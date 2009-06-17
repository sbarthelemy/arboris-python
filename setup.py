# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from distutils.core import setup, Command
from distutils.extension import Extension
from Cython.Distutils import build_ext
from sphinx.setup_command import BuildDoc


from glob import glob
import os
from os.path import splitext, basename, join as pjoin, walk, sep
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
    """
    Copy __doc__ to __test__ to workaround doctest failure with pyx files
    """
    module.__test__ = {}
    for name in dir(module):
       value = getattr(module, name)
       if inspect.isbuiltin(value) and \
          isinstance(value.__doc__, str) and \
          _from_module(module, value):
           module.__test__[name] = value.__doc__

class SphinxCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        os.chdir('doc')
        pid = os.fork()
        if not pid:
            os.execvp("make",['', 'html'])
        os.wait()


class TestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        '''
        Finds all the tests files in tests/, and run doctest on them.
        '''
        pymods = []
        pyxmods = []
        for root, dirs, files in os.walk('arboris'):
            for file in files:
                package = '.'.join(root.split(sep))
                if file.endswith('.py') and file != '__init__.py':
                    pymods.append('.'.join([package, splitext(file)[0]]))
                elif file.endswith('.pyx'):
                    pyxmods.append('.'.join([package, splitext(file)[0]]))

        for mod in pymods:
            exec('import {0} as module'.format(mod))
            doctest.testmod(module)

        for mod in pyxmods:
            exec('import {0} as mod'.format(mod))
            fix_module_doctests(mod)
            doctest.testmod(mod)

        for rst in glob(pjoin('tests', '*.rst')):
            doctest.testfile(rst)

version = '0.1.0pre2'

setup(
    name='arboris',
    author = 'Sébastien BARTHÉLEMY',
    version = version,
    packages=['arboris',
              'arboris.robots'],
    cmdclass = {'build_ext': build_ext, 'test': TestCommand, 
                'sphinx': SphinxCommand, 'build_sphinx': BuildDoc},
    ext_modules = [Extension("arboris.misc_c",
                             ["arboris/misc_c.pyx"]),
                  Extension("arboris.homogeneousmatrix",
                             ["arboris/homogeneousmatrix.pyx"]),
                   Extension("arboris.twistvector",
                             ["arboris/twistvector.pyx"])
                  ]
)

