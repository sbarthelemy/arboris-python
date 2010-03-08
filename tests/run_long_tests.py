# coding=utf-8
"""Run all the long tests."""

import arboris
import unittest
import doctest
import glob
import os

_loader = unittest.defaultTestLoader
suite = unittest.TestSuite()
for rst in glob.glob(os.path.join('tests','*.rst')):
    suite.addTest(doctest.DocFileSuite(os.path.basename(rst)))
suite.addTest(_loader.loadTestsFromName('test_energy_drift'))

if __name__ == "__main__":
    unittest.TextTestRunner(verbosity=2).run(suite)
