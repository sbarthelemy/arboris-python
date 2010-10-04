# coding=utf-8
"""Run all the unit tests."""
import unittest
import arboristest

_loader = unittest.defaultTestLoader
suite = unittest.TestSuite()
suite.addTest(_loader.loadTestsFromName('test_doctests.suite'))
suite.addTest(_loader.loadTestsFromName('test_joints'))
suite.addTest(_loader.loadTestsFromName('test_update_dynamic'))
suite.addTest(_loader.loadTestsFromName('test_constraints'))
suite.addTest(_loader.loadTestsFromName('test_visu_collada'))

if __name__ == "__main__":
    arboristest.main(defaultTest='suite')
