# coding=utf-8
"""Run all doctests from the arboris module."""

import unittest
import doctest
import glob
import os

def doctest_package(suite, package_name):
    """Recursively add the doctests from a package.
    """
    exec('import {0} as package'.format(package_name))
    for module_name in package.__all__:
        exec('from {0} import {1} as module'.format(package_name, module_name))
        try:
            suite.addTest(doctest.DocTestSuite(module))
        except ValueError as e:
            if len(e.args)>=2 and e.args[1]=="has no tests":
                pass
            else:
                raise e
        try:
            module.__all__ # raise Attribute error if not a package
            doctest_package(suite, package_name + '.' + module_name)
        except AttributeError:
            pass

suite = unittest.TestSuite()
doctest_package(suite, 'arboris')

if __name__ == "__main__":
    unittest.TextTestRunner(verbosity=2).run(suite)
