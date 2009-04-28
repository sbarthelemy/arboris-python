# coding: utf-8
"""This script makes a zipped snapshot from the git repository.

TODO: add exception handling
TODO: make doctests first
TODO: move to scons

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

import os
import shutil
import sys
import glob


def test(args):

    for f in glob.glob('./src/*.py'):
        pid = os.fork()
        if not pid:
            os.execvp("python", ['',f])
        os.wait()



if __name__ == "__main__":
    test(sys.argv[1:])
    
