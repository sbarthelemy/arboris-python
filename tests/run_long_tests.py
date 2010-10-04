# coding=utf-8
"""Run all the long tests."""

import arboris
import unittest
import arboristest
import doctest
import glob
import os

_loader = unittest.defaultTestLoader
suite = unittest.TestSuite()
for rst in glob.glob(os.path.join('tests','*.rst')):
    suite.addTest(doctest.DocFileSuite(os.path.basename(rst)))
suite.addTest(_loader.loadTestsFromName('test_energy_drift'))
suite.addTest(_loader.loadTestsFromName('test_human36_falling'))
suite.addTest(_loader.loadTestsFromName('test_human36'))
suite.addTest(_loader.loadTestsFromName('test_pdcontroller'))

if __name__ == "__main__":
    arboristest.main(defaultTest='suite')
