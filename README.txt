
Arboris is a rigid body dynamics and contacts simulator written in python.

Arboris includes a generic and easily extensible set of joints
(singularity-free multi-dof joints, non-honolomic joints, etc.) which are
used to model open rigid mechanisms with a minimal set of state variables.

The dynamics of these systems are computed in a form similar to the
Boltzmann-Hamel equations.

Using time-stepping and a semi-implicit Euler integration scheme, a first-order
approximation of the model is also computed. This allows for additional
constraints such as contacts and kinematic loops to be solved using a
Gauss-Seidel algorithm.

Arboris is mostly useful for robotic applications and human motion studies.
The python language makes it particularly suited for fast-paced development
(prototyping) and education.


Background
==========

In 2005, Alain Micaelli, a researcher from CEA LIST, wrote a first version of
the simulator in the matlab language. It was an implementation (and often an
extension) of the algorithms described in [Park2005]_, [Murray1994]_ and
[Liu2003]_.
He was later joined by Sébastien Barthélemy, from ISIR/UPMC, who reorganized
the code to take advantage of the early object-oriented features of matlab.
It eventually became clear that the language was ill-designed, and that a full
rewrite was necessary. With the help of Joseph Salini, also from ISIR/UPMC,
Arboris-python was born. The resulting framework is now quite similar to what
is presented in [Duindam2006]_.

The matlab version of the simulator is now deprecated.

.. [Murray1994]
    Richard M. Murray, Zexiang Li and S. Shankar Sastry,
    "A  Mathematical Introduction to Robotic Manipulation",
    CRC Press, 1994.

.. [Park2005]
    Jonghoon Park,
    "Principle of Dynamical Balance for Multibody Systems",
    Multibody System Dynamics, vol. 14, number 3-4, pp. 269-299, 2005.

.. [Liu2003]
    T. Liu and M. Y. Wang,
    "Computation of three dimensional rigid body dynamics of
    multiple contacts using time-stepping and Gauss-Seidel
    method",
    IEEE Transaction on Automation Science and Engineering,
    submitted, November 2003.

.. [Duindam2006]
    V. Duindam,
    "Port-Based Modelling and Control for Efficent Bipedal Walking Robots",
    University of Twente, 2006.
