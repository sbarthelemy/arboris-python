=============
Development
=============

This is a small page to help hacking arboris-python.
  
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

