=========
Notations
=========

A notation conversion table
===========================

=======================  =======================  =======================  =======================  =======================  ============
arboris-python           stramigioli icra2001     duindam 2006             li & sastry              KDL (cf frames.hpp)
=======================  =======================  =======================  =======================  =======================  ============
`\Hg[i]_j`               `H^i_j`                  `H^i_j`                  `g_{ij}`                 F_A_B
`\twist[k]_{j/i}`        `T^{k,i}_j` (p24)                                                          T                        twist of `j` with respect to `i` expressed in `k`
`\twist[j]_{j/i}`        `T^{j,i}_j`                                                                                         body twist
`\twist[i]_{j/i}`        `T^{i,i}_j`                                                                                         "world" twist
                                                                                                    W                        wrench
                                                                                                    R                        rotation
                                                                                                    V_A                      vector expressed in A
`\Ad[i]_j`                                                                 `\text{Ad}_{g_{ij}}`
=======================  =======================  =======================  =======================  =======================  ============

=======================  =======================  ==========================
documentation            latex                    sourcecode
=======================  =======================  ==========================
`\Hg[i]_j`               ``\Hg[i]_j``              ``H_ij``
`\Ad[i]_j`               ``\Ad[i]_j``             ``Ad_ij``
`\twist[j]_{j/i}`        ``\twist[j]_{j/i}``      ``T_ji``
`\J[j]_{j/i}`            ``\J[j]_{j/i}``          ``J_ji``
`\dJ[j]_{j/i}`           ``\dJ[j]_{j/i}``         ``dJ_ji``
=======================  =======================  ==========================

Some formulas
=============

.. math::

  \Hg[i]_j(t) &= \exp(t \twist[j]_{j/i}) \; \Hg[i]_j(0) \\
             &= \Hg[i]_j(0) \exp(t \twist[i]_{j/i})
 
How KDL works
=============

`Twist` class::
	twist.rot
	twist.vel

``Wrench` class::
	wrench.force
	wrench.forque

`twist(i)` returns `hcat(vel, rot)[i]` and `wrench(i)` returns `hcat(force, torque)[i]`

a frame object contains a rotation matrix `frame.M` and an origin vector `frame.p`. 
The `*` is overload, in order to works as expected with vectors points, twists, wrenches 
and other frames.
