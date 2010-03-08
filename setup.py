# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from distutils.core import setup, Command
from glob import glob
import os
from os.path import splitext, basename, join as pjoin, walk, sep
import doctest

cmdclass = {}
try:
    # add a command for building the html doc
    from sphinx.setup_command import BuildDoc
    cmdclass['build_doc'] = BuildDoc
except ImportError:
    pass

version = '0.1.0pre4'
readme = open('README.txt', 'r')

setup(name='arboris',
      version=version,
      maintainer='S√©bastien BARTH√âLEMY',
      maintainer_email='barthelemy@crans.org',
      url='https://vizir.robot.jussieu.fr/trac/arboris',
      description='A rigid body dynamics and contacts simulator written in python.',
      long_description=readme.read(),
      license='LGPL',
      packages=['arboris',
                'arboris.robots'],
      classifiers=[
              'Development Status :: 4 - Beta',
              'Environment :: Console',
              'Intended Audience :: Science/Research',
              'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
              'Operating System :: OS Independent',
              'Programming Language :: Python',
              'Topic :: Scientific/Engineering :: Physics'],
      cmdclass=cmdclass)
readme.close()
