===============
Simulation loop
===============

Overview
========

A simulation can be ran using the :func:`arboris.core.simulate` function, 
which is indeed very simple::

    def simulate(world, time):
        world.init()
        previous_t = time[0]
        for t in time[1:]:
            dt = t - previous_t
            world.update_dynamic()
            world.update_controllers(dt, t)
            world.update_constraints(dt)
            world.integrate(dt)
            previous_t = t    

Let's detail the various steps.


:meth:`~arboris.core.World.update_dynamic`
==========================================

The :meth:`~arboris.core.World.update_dynamic` method computes the 
matrices of the world model in generalized coordinates. We'll denote
respectively `M`, `B` and `N` the mass, viscosity and non-linear effects 
matrices.

At this step, the world "unactuated" model is 

.. math::
    M(t) \dGVel(t+dt) + \left( N(t) + B(t) \right) \GVel(t+dt) = 0,

where `\GVel` is the generalized velocity vector and `\dGVel` its time 
derivative.

Note that there is no contact, actuation or even gravity involved yet. 


:meth:`~arboris.core.World.update_controllers`
==============================================

The :meth:`~arboris.core.World.update_controllers` method completes the 
model with "actuation" forces provided by the controllers. Actuation is 
here taken in a broaden meaning and may actually include weight (from 
:class:`arboris.controllers.WeightController`).

Considering the following integration scheme
        
.. math::
    \dGVel(t+dt) = \frac{\GVel(t+dt) - \GVel(t)}{dt}
        
we get this first order free model
        
.. math::
    \left( \frac{M(t)}{dt} + N(t) + B(t) \right) \GVel(t+dt) &= 
    \frac{M(t)}{dt} \GVel(t)

which is then completed by the controllers first-order actuation force:

.. math::
    \GForce_a(t) &= \GForce_{0a}(t) + Z_a(t) \GVel(t+dt)

It is the responsability of each controller to provide both `\GForce_{0a}` and
`Z_a(t)` when its :meth:`~arboris.core.Controller.update` method is called by
:meth:`~arboris.core.World.update_controllers`.

The actuated model is then

.. math::
    \left( \frac{M(t)}{dt} + N(t) + B(t) - \sum_a Z_a(t) \right) 
    \GVel(t+dt) &= 
    \frac{M(t)}{dt} \GVel(t) + \sum_a \GForce_{0a}(t)

Introducing the world impedance `Z` and admittance `Y`.

.. math::
    Z(t) &= \frac{M(t)}{dt} + N(t) + B(t)- \sum_a Z_a(t) \\
    Y(t) &= Z^{-1}(t)

leads to a more compact expression:

.. math::
    \GVel(t+dt) 
    &= Y(t) \left( \frac{M(t)}{dt} \GVel(t) + \sum_a \GForce_{0a}(t) \right)

The role of the :meth:`~arboris.core.World.update_controllers` method
is to compute `Z`, `Y` and `\sum_a \GForce_{0a}` and to save them 
respectively in the world :attr:`~arboris.core.World._impedance`, 
:attr:`~arboris.core.World._admittance` and 
:attr:`~arboris.core.World._gforce` properties.


:meth:`~arboris.core.World.update_constraints`
==============================================

Additionnal constraints such as contacts or kinematic loops are handled by 
the :meth:`~arboris.core.World.update_constraints` method which solves 
them iteratively using a Gauss-Seidel algorithm. It results in a constant 
generalized force `\GForce_{c}` for each constraint which is added to 
the actuation ones in the world :attr:`~arboris.core.World._gforce` property.

The resulting model is:

.. math::
    \GVel(t+dt) &= Y(t) 
    \left( 
        \frac{M(t)}{dt} \GVel(t) 
        + \sum_a \GForce_{0a}(t) 
        + \sum_c \GForce_{c}(t)
    \right)


:meth:`~arboris.core.World.integrate`
=====================================

From the previous equation, this method computes `\GVel(t+dt)` and then 
calls each joint `j` :meth:`~arboris.core.Joint.integrate` method which 
updates the joint generalized position `\GPos_j(t+dt)`.

That's it, the world state (generalized positions and velocities) has 
been updated to `t+dt` and all the model matricies are now outdated.

