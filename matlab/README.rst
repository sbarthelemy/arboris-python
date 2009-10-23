This directory contains tools to convert robots and simulations between
arboris-matlab and arboris-python (in both ways) and to compare the
simulations results.

This is useful when porting a robot or a controller from one simutator 
to the other, or to check against regressions when a bug is found.

Yet, the conversion tools are not complete nor well tested, use them 
with caution.


The scripts are intended to be ran from this very directory. The 
simulators should be in your matlab/python paths.

For matlab, you could create a ''startup.m'' file in the directory,
with its content inspired from::

  addpath('/home/seb/my/arboris/path/')
  arb_setpath;
