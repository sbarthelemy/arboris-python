# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from distutils.core import setup, Command
from glob import glob
import os
from os.path import splitext, basename, join as pjoin, walk, sep
import doctest

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
                elif file.endswith('.so'):
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


cmdclass = {'test': TestCommand}

try:
    from sphinx.setup_command import BuildDoc
    cmdclass['build_doc'] = BuildDoc
except ImportError:
    pass

version = '0.1.0pre3'

setup(
    name='arboris',
    author='Sébastien BARTHÉLEMY',
    version=version,
    packages=['arboris',
              'arboris.robots'],
    cmdclass=cmdclass,
    package_data = {'arboris':['doc/']}
)

