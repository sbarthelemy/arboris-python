============
Introduction
============

.. include:: ../README.txt

Comparison to arboris-matlab
============================

I believe this version of arboris, written in python, is much better 
than the one written in matlab, and I'll try here to explain why.


- python is better than matlab for what we do

  - arboris-matlab is written in the first (pre r2008a) matlab 
    objet-oriented language, which is a really bad language. 
    So bad that an entire book has been written about various 
    tricks to circumvent its flaws (serious flaws as: no reference, 
    no namespace, everything-is-matrix-philosophy-but-not-always, arrays 
    cannot be subclassed...). 
    I do think that if the matlab version is to be kept, it 
    should be rewritten in the new matlab object-oriented 
    language.

  - python is cheaper than matlab,

  - python has lots useful development tools such as docutils, doctest, 
    sphinx. (Well, matlab has an integrated debugger which python still 
    misses).

  - More good (free) libraries are available for python than for matlab 
    (in particular thanks to `swig <http://www.swig.org>`_,
    `sage <http://www.sagemath.org>`_). 
    (Matlab has really good toolboxes too, but the price tag is not the 
    same).

  - Python, as a scripting language, is embedded in blender and maya, which 
    opens up great visualisation possibilities.

  - its easier to wrap C/C++ code to python than to matlab.

- arboris-python design is better, because it has been written from the 
  ground up, keeping the good ideas from the matlab version, but not the
  historical mess of code. Here are some more specific reasons:

  - the tree of bodies and joints is described in a tree datastructure, not
    as a combination of serial chains.

  - generic joints handling is better. It know possible to have planar models, 
    anchored robots and singularity-free ball and socket joints.

  - the visualisation is clearly separated from the core.

  - the controllers model is first order, without modifying the whole 
    robot/world.

  - the simulation is faster, because the design is better and because 
    in arboris-python we do not integrate a second time after the 
    constraints computation.

- misc

  - the arboris-python version has a much better documentation.

  - there is no more ``error('coucou')`` messages

  - the arboris-python version has automated unit tests.



There are still areas where arboris-python is behind:

- python as no integrated debugger,
- fewer collisions are implemented (it's easy to fix),
- fewer contact models are implemented (it's not hard to fix),
- there is no visco-elastic constraint,
- there is no kinematic (or 0-degree) robot,
- the python version has been less tested by real users,
- the python version has no gui to set the posture.
- there is no more ``error('coucou')`` messages

