=========
Notations
=========

A notation conversion table
===========================

=======================  =======================  =======================  =======================  ============
arboris-python           stramigioli icra2001     duindam 2006             li & sastry
=======================  =======================  =======================  =======================  ============
:math:`\Hg[i]_j`         :math:`H^i_j`            :math:`H^i_j`            :math:`g_{ij}`
:math:`\twist[k]_{j/i}`  :math:`T^{k,i}_j` (p24)                                                    twist of j with respect to i expressed in k
:math:`\twist[j]_{j/i}`  :math:`T^{j,i}_j`                                                          body twist
:math:`\twist[i]_{j/i}`  :math:`T^{i,i}_j`                                                          "world" twist
=======================  =======================  =======================  =======================  ============


Some formulas
=============

.. math::

  \Hg[i]_j(t) &= \exp(t \twist[j]_{j/i}) \; \Hg[i]_j(0) \\
              &= \Hg[i]_j(0) \exp(t \twist[i]_{j/i})
 
