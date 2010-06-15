# coding=utf-8
"""Run all doctests from the arboris module."""

import arboris
import unittest
import doctest
import glob
import os

suite = unittest.TestSuite()
for module_name in arboris.__all__:
    exec('from arboris import {0} as module'.format(module_name))
    try:
        suite.addTest(doctest.DocTestSuite(module))
    except ValueError as e:
        if len(e.args)>=2 and e.args[1]=="has no tests":
            pass
        else:
            raise e

if __name__ == "__main__":
    unittest.TextTestRunner(verbosity=2).run(suite)
