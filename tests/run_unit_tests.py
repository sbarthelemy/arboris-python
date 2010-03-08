# coding=utf-8
"""Run all the unit tests."""
import unittest

_loader = unittest.defaultTestLoader
suite = unittest.TestSuite()
suite.addTest(_loader.loadTestsFromName('test_doctests.suite'))
suite.addTest(_loader.loadTestsFromName('test_joints'))
suite.addTest(_loader.loadTestsFromName('test_update_dynamic'))

if __name__ == "__main__":
    unittest.TextTestRunner(verbosity=2).run(suite)
