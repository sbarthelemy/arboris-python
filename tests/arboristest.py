# -*- encoding: UTF-8 -*-

from unittest import TestCase
from numpy import ndarray


class TestCase(TestCase):

    def assertListsAlmostEqual(self, seq1, seq2, places=7):
        sequence = (tuple, list, ndarray)
        if len(seq1) != len(seq2):
            raise AssertionError("%s != %s"%(str(seq1), str(seq2)))
        for i, j in zip(seq1, seq2):
            if isinstance(i, sequence) and isinstance(j,  sequence):
                self.assertListsAlmostEqual(i, j, places)
            else:
                self.assertAlmostEqual(i, j, places)
