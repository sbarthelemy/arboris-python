# coding=utf-8
"""Run all doctests from the arboris documentation."""

import unittest
import doctest
import glob
import os

all = unittest.TestSuite()
for f in glob.glob('doc/*.rst'):
    suite = doctest.DocFileSuite(os.path.abspath(f), module_relative=False)
    all.addTest(suite)
    vars()[os.path.basename(f).split('.')[0]] = suite
del(suite)

if __name__ == "__main__":
    unittest.main(defaultTest='all')
