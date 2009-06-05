from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

setup(
    name='arboris',
    packages=['arboris',
 	          'arboris.robots'],
    package_dir = {'arboris':'src'},
    cmdclass = {'build_ext': build_ext},
    ext_package='arboris',
    ext_modules = [Extension("homogeneousmatrix",
                             ["src/homogeneousmatrix.pyx"])]
)

