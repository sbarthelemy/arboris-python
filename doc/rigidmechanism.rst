================
Rigid Mechanisms
================

We define a rigid mechanism as a finite number of rigid bodies interconnected by ideal joints.

Ideal Joints
============

An ideal joint is a kinematic restriction of the allowed relative twist of two rigid bodies :math:`i` and :math:`j` to a linear subspace of dimension :math:`k`, where the relative motion of the bodies is described by two set of states, namely 

- a matrix :math:`\GPos`, parameterizing the relatie configuration as :math:`\Hg[i]_j = \Hg[i]_j(\GPos)`,
- a vector :math:`\GVel \in \Re^k`, parameterizing the relative twist as :math:`\twist[i]_{i/j} = X(\GPos) \GVel`

where :math:`X(\GPos)` depends smoothly on :math:`\GPos` and :math:`\nu = V_\GPos(\dot{\GPos})` with :math:`V_\GPos` invertible and linear in :math:`\dot{\GPos}`. Furthermore, there exists a mapping :math:`F_\GPos : \Re^k \rightarrow \GPosSet`.




