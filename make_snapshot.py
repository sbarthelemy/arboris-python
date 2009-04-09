#! python
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

tmp_dir = '/tmp/'

def build_html_doc():
    os.chdir('doc')
    pid = os.fork()
    if not pid:
        os.execvp("make",["","html"])
    os.wait()
    os.chdir('..')



def make_snapshot(name):

    build_html_doc()

    target_dir = os.path.join(tmp_dir,name)
    os.mkdir(target_dir)
    src_target_dir = os.path.join(target_dir,'python-arboris')
    os.mkdir(src_target_dir)
    for f in glob.glob('./src/*.py'):
        shutil.copy(f, src_target_dir)
    shutil.copytree("doc/.build/html", 
                    os.path.join(target_dir, "doc-html"))

    os.chdir(tmp_dir)
    pid = os.fork()
    if not pid:
        os.execvp("zip",['', '-r', name+'.zip', name])
    os.wait()



if __name__ == "__main__":
    name = sys.argv[1]
    make_snapshot(name)
    
