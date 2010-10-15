=============
Development
=============

This is a small page to help hacking arboris-python.

This software is written in the python language, version 2.6. On the contrary
to python 2.7, the version 2.6 was shipped with the Lucid Lynx (10.04) release
of Ubuntu, which will be supported until april 2013.

Using Git
=========

Linux setup
-----------

Install and set up git::

  sudo aptitude install git-gui gitk
  git config --global user.name "Your Name Comes Here"
  git config --global user.email you@yourdomain.example.com
  git config --global --add color.diff auto
  git config --global --add color.interactive always

Then, run ``git help tutorial`` for help.

Workflow
--------

Seb, as the first author of arboris, creates a repository on its laptop::

  seb@seb-laptop$ mkdir arboris-python
  seb@seb-laptop$ cd arboris-python
  seb@seb-laptop$ git init
  seb@seb-laptop$ edit ...files...
  seb@seb-laptop$ git add ...files...
  seb@seb-laptop$ git commit

Then, in order to make the repository accessible to others, Seb puts it on the vizir server::

  seb@seb-laptop$ scp -r  arboris-python seb@vizir.robot.jussieu.fr:
  seb@seb-laptop$ ssh seb@vizir.robot.jussieu.fr
  seb@vizir$ git clone --bare arboris-python arboris-python.git
  seb@vizir$ rm -rf arboris-python
  seb@vizir$ cd arboris-python.git
  seb@vizir$ git config receive.denyNonFastforwards true
  seb@vizir$ logout
  seb@seb-laptop$ rm -rf arboris-python
  seb@seb-laptop$ git clone ssh://seb@vizir.robot.jussieu.fr/home/seb/arboris-python.git

Now, Seb can work locally and push back to vizir::

  TODO: explain how

Joe wants to help. He can fetch Seb's repository, and produce a patch::

  joe@joe-laptop$ git clone ssh://salini@vizir.robot.jussieu.fr/home/seb/arboris-python.git
  joe@joe-laptop$ cd arboris-python
  joe@joe-laptop$ edit ...files... #(Joe improves the visualization)
  joe@joe-laptop$ git add ...files...
  joe@joe-laptop$ git commit
  joe@joe-laptop$ git diff master..origin/master > visu-impr.patch

Then he sends the patch to Seb by email, who applies it and push the result back to vizir::

  seb@seb-laptop$ git apply visu-impr.patch
  seb@seb-laptop$ git add ...files...
  seb@seb-laptop$ git commit
  seb@seb-laptop$ git push

Eventually, when Joe issues a new pull, everything gets merged gracefully::

  joe@joe-laptop$ git pull

Tests
=====

Overview
--------

Arboris comes with several tests, located in different places, and using
different tools, but all are run from the unittest framework.

1. test cases using the unittest frameworks, located in
   :file:`/tests/test_{*}.py`.
   Some of these use a customized version of the :class:`TestCase` class,
   defined in the :mod:`arboristest` module, which is located in
   :file:`/tests/arboristest.py`. They are normaly suitable for batch
   processing, but the ``--interactive`` option can be passed to let a test
   run animations or require user interaction.
   All these test are run either from either :file:`/tests/run_unit_tests.py`
   or :file:`/tests/test_long_tests.py`.

2. doctests in :file:`/tests/test_{*}.rst` files, ran as unittest jobs from
   :file:`run_long_tests.py`,

3. doctests located in docstrings, ran as unittest jobs from
   :file:`/tests/test_docstest.py`, itself ran from
   :file:`/tests/run_unit_tests.py`.

4. doctests located in documentation files, ran as unittest jobs from
   :file:`/test/run_documentation_tests.py`.

Running all the tests
---------------------

::

    python test_unit_tests.py
    python test_long_tests.py
    python test_documentation_tests.py

Unittest customizations
-----------------------

.. automodule:: arboristest
   :members:
   :undoc-members:

Using pylint
============

One can run `Pylint <http://www.logilab.org/857>`_ to look for coding style
violations etc.::

  pylint --rcfile=.pylintrc -f html arboris/ > pylint_report.html

Building the documentation
==========================

::

    python setup.py build_doc

