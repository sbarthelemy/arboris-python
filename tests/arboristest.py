# -*- encoding: UTF-8 -*-

import unittest
import os
import sys
import tempfile
import optparse
from numpy import ndarray

class TestCase(unittest.TestCase):
    """A class whose instances are single test cases.

    This is a variant of :class:`unittest.TestCase` which adds:

    the :meth:`assertListsAlmostEqual` method to compare sequences of floating
    points numbers

    The following attributes:

    - :attribute:`testdir`: the directory where the reference result files are
      stored,

    - :attribute:`destdir`: the path of a place where temporary result files
      should be written (this defauts to a temporary directory),

    - :attribute:`interactive`: (boolean) if False (the default), the test case
      should not require user interaction (such as displaying an animation) and
      therefore be suitable for batch processsing.

    """
    interactive = False
    destdir = tempfile.gettempdir()
    testdir = os.path.abspath(os.path.dirname(__file__))

    def assertListsAlmostEqual(self, seq1, seq2, places=7):
        sequence = (tuple, list, ndarray)
        if len(seq1) != len(seq2):
            raise AssertionError("%s != %s"%(str(seq1), str(seq2)))
        for i, j in zip(seq1, seq2):
            if isinstance(i, sequence) and isinstance(j,  sequence):
                self.assertListsAlmostEqual(i, j, places)
            else:
                self.assertAlmostEqual(i, j, places)

class TestProgram(unittest.TestProgram):
    """
    A command-line program that runs a set of tests; this is primarily
    for making test modules conveniently executable.

    This version modifies :func:`unittest.main` to add the
    ``--destdir`` and ``--interactive`` options which are specific to
    arboris.

    """
    USAGE = """\
Usage: %(progName)s [options] [test] [...]

Examples:
  %(progName)s                               - run default set of tests
  %(progName)s MyTestSuite                   - run suite 'MyTestSuite'
  %(progName)s MyTestCase.testSomething      - run MyTestCase.testSomething
  %(progName)s MyTestCase                    - run all 'test*' test methods
                                               in MyTestCase
"""
    def parseArgs(self, argv):
        import getopt
        parser = optparse.OptionParser(self.USAGE)
        # options specific to arboris
        parser.add_option("-d", "--destdir",
                action="store", type="string", dest="destdir",
                help="write result files (if any) to DIR [default: %default]",
                metavar="DIR",
                default=TestCase.destdir)
        parser.add_option("-i", "--interactive", default=TestCase.interactive,
                action="store_true", dest="interactive",
                help="allow test to require human intervention")
        # options for unittest
        parser.add_option('-q', '--quiet',
                action="store_true",
                help="minimal output")
        parser.add_option('-v', '--verbose',
                action="store_true",
                help="verbose output")
        (options, args) = parser.parse_args(sys.argv[1:])

        TestCase.interactive = options.interactive
        TestCase.destdir = options.destdir

        if len(args) == 0 and self.defaultTest is None:
            self.test = self.testLoader.loadTestsFromModule(self.module)
            return
        if len(args) > 0:
            self.testNames = args
        else:
            self.testNames = (self.defaultTest,)
        self.createTests()

main = TestProgram
