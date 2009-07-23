# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

"""

.. todo:
    merge Constraint and BodyConstraint classes? This will depend on the joint
    stops constraints implementation

"""

from abc import ABCMeta, abstractmethod
from numpy import array, zeros, eye, dot, hstack, diag
from numpy.linalg import solve, eigvals
import homogeneousmatrix as Hg
from core import SubFrame, Constraint, Shape, NamedObject

point_contact_proximity = 0.02
joint_limits_proximity = 0.01

class JointLimits(Constraint):
    r"""This class describes and solves joint limits constraints.

    Let's denote `q` and `\GVel` the joint generalized position and velocity,
    `m` and `M` the joint limits. This class purpose is to enforce the 
    following constraint on `q`

    .. math::
        m \leq q \leq M

    through a generalized force `\GForce`, while ensuring the Signorini
    conditions holds

    .. math::
        0 \leq q- m \perp \GForce \geq 0  \\
        0 \geq q - M \perp \GForce \leq 0

    
    """

    def __init__(self, joint, min, max, proximity=None, 
                 name=None):
        from arboris.joints import LinearConfigurationSpaceJoint
        if not isinstance(joint, LinearConfigurationSpaceJoint):
            raise ValueError()
        self._joint = joint
        NamedObject.__init__(self, name)
        self._min = array(min).reshape((joint.ndof,))
        self._max = array(max).reshape((joint.ndof,))
        if proximity is None:
            #TODO: choose a proper default for tol according to the joint type
            self._proximity = zeros((joint.ndof,))
            self._proximity[:] = joint_limits_proximity
        else:
            self._proximity = array(proximity).reshape((joint.ndof,))
        self._pos0 = None
        self._jacobian = None
        self._force = zeros((joint.ndof,))

    def init(self, world):
        self._jacobian = zeros((1, world.ndof))
        self._jacobian[0,self._joint._dof] = 1

    @property
    def jacobian(self):
        return self._jacobian
    
    @property
    def ndol(self):
        return 1

    def update(self):
        self._pos0 = self._joint.gpos
    
    def is_active(self):
        return (self._pos0-self._min<self._proximity) or \
                (self._max-self._pos0<self._proximity)    

    def solve(self, vel, admittance, dt):
        pred = self._pos0 + dt*vel
        # pos = self._pos0 + dt*(vel + admittance*dforce)
        if (pred <= self._min):
            # the min limit is violated, we want pos == min
            dforce = solve(admittance, (self._min - pred)/dt)
            self._force += dforce

        elif (self._max <= pred):
            #the max limit is violated, we want pos == max
            dforce = solve(admittance, (self._max - pred)/dt)
            self._force += dforce

        else:
            # the joint is within its limits, we ensure the 
            # generalized force is zero
            dforce = -self._force
            self._force[:] = 0.

        return dforce

class BallAndSocketConstraint(Constraint):
    r"""
    This class describes and solves a ball and socket kinematic constraint
    between two frames which are rigidly fixed to two distinct bodies.

    Let's denote 0 and 1 these two frames. The constraint can be expressed
    as a condition on their relative pose, requiring  than the ball 
    and socket centers be co-located:

    .. math::
        
        \Hg[0]_1 = 
        \begin{bmatrix}
            \pre[0]R_1 & 0\\
            0          & 1
        \end{bmatrix}
        \Leftrightarrow 
        \pre[0]p_1 = 0

    Solving the constraint means computing the right constraint force that will
    ensure that the kinematic condition will be met.

    Let's define the constraint velocity `v` and the constraint force
    `f` as

    .. math::

        v &= S \; \twist[0]_{1/0} = \pre[0]{\dot{p}}_1 \\
        \wrench[0]_{0/1} &= S^T \; f  \\
        S &= 
        \begin{bmatrix}
        0 & 0 & 0 & 1 & 0 & 0\\
        0 & 0 & 0 & 0 & 1 & 0\\
        0 & 0 & 0 & 0 & 0 & 1
        \end{bmatrix} 

    The constraint jacobian is then given by 
    `( \Ad[0]_1 \; \pre[1]J_{1/g} - \pre[0]J_{0/g} )`
 
    In order to solve the constraint, an adjustement `\Delta f` of the
    constraint force is computed by the ``solve`` method. 
    This method is given as argument the current estimation of constraint
    velocity, which corresponds to `\Delta f = 0` and takes into account
    the coupling between the contraints.

    .. math::
        f^k(t) 
        &= f^{k-1}(t) + \Delta f\\
        v^k(t+dt)
        &= v^*(t+dt) + Y(t) \Delta f  \\
        \Hg[0]_{1/0}^k(t+dt)
        &= \exp( dt \; S^T \; ( v^*(t+dt) + Y(t) \; \Delta f ) ) \;
        \Hg[0]_{1/0}(t)
        \\
        \pre[0]p_1^k(t+dt)
        &= \pre[0]p_1(t) + dt \; (v^*(t+dt) +  Y(t) \; \Delta f) 

    """
   
    def __init__(self, frames, name=None):
        self._force = zeros(3)
        self._pos0 = None
        NamedObject.__init__(self, name)
        self._frames = frames

    def init(self, world):
        pass

    @property
    def ndol(self):
        return 3

    def update(self):
        r"""
        Compute the predicted relative position error between the socket and
        ball centers, `\pre[0]p_1(t)` and save it in ``self._pos0``.

        .. math::

            \Hg[0]_1(t) 
            &= \left(\Hg[g]_0(t)\right)^{-1} \; \Hg[g]_1(t) \\
            \Hg[0]_1(t) 
            &=
            \begin{bmatrix}
            \pre[0]R_1(t) & \pre[0]p_1(t) \\
            0             & 1
            \end{bmatrix}

        """
        H_01 = dot(Hg.inv(self._frames[0].pose), self._frames[1].pose)
        self._pos0 = H_01[0:3,3]

    def is_active(self):
        return True

    @property
    def jacobian(self):
        H_01 = dot(Hg.inv(self._frames[0].pose), self._frames[1].pose)
        return (dot(Hg.adjoint(H_01)[3:6,:], self._frames[1].jacobian)
                -self._frames[0].jacobian[3:6,:])

    def solve(self, vel, admittance, dt):
        r"""

        from this equation
        
        .. math::
            \pre[0]p_1^k(t+dt) 
             &= \pre[0]p_1(t) + dt \; ( v^* +  Y \; \Delta f) 

        given that we want `\pre[0]p_1(t+dt) = 0`, we return 

        .. math::
            \Delta f = -Y^{-1} \; 
            \left(
                \frac{\pre[0]p_1(t)}{dt} + v^* 
            \right)

        and update the constraint force adjustment `\Delta f` 

        The function arguments are

        - ``vel``: `v^*`
        - ``admittance``: `Y`
        - ``dt``: `dt`

        Tests:

        >>> c = BallAndSocketConstraint(frames=(None, None))
        >>> c._pos0 = array([0.1, 0.2, 0.3])
        >>> c._force = array([-0.1, -0.2, -0.3])
        >>> vel = zeros((3))
        >>> adm = 0.5*eye(3)
        >>> dt = 0.1
        >>> dforce = c.solve(vel, adm, dt)
        >>> c._pos0 + dt * ( vel + dot(adm, dforce) )
        array([ 0.,  0.,  0.])
        >>> vel = array([0.05, -0.05, 0.05])
        >>> dforce = c.solve(vel, adm, dt)
        >>> c._pos0 + dt * ( vel + dot(adm, dforce) )
        array([ 0.,  0.,  0.])

        """
        dforce = solve(-admittance, vel + self._pos0/dt)
        self._force += dforce
        return dforce


class PointContact(Constraint):
    r"""Parent class for all point contacts.
    
    Solving a contact constraint involves two steps:

    - the ``update`` method does the collision detection then updates
      the contacts frames,

    - the ``solve`` method, called at each iteration of the
      Gauss-Seidel algorithm computes contact forces compatible 
      with the contact model.

    The :class:`PointContact` implements the first step by calling a
    ``collision_solver`` function. Implementing the second one is the 
    responsability of a daughter class.

    """

    def __init__(self, shapes, collision_solver, proximity):
        r"""
        .. todo:
            find the good ``collision_solver`` automatically according
            to the ``shapes`` pair
        """
        assert isinstance(shapes[0], Shape)
        assert isinstance(shapes[1], Shape)
        if collision_solver is None:
            # automatically find the collision solver
            from collisions import choose_solver
            (shapes, collision_solver) = choose_solver(shapes[0], shapes[1])
        self._shapes = shapes
        self._frames = (SubFrame(shapes[0].frame.body),
                        SubFrame(shapes[1].frame.body))
        self._collision_solver = collision_solver
        self._proximity = proximity

    def init(self, world):
        pass

    def update(self):
        r"""
        This method calls the collision solver and updates the constraint
        status and the contact frames poses accordingly.
        The two frames have the same orientation, with the contact normal along
        the `z`-axis
        """
        (sdist, H_gc0, H_gc1) = self._collision_solver(self._shapes)
        #self._frames[0].bpose = dot(self._shapes[0].frame.bpose, H_s0c0)
        #self._frames[1].bpose = dot(self._shapes[1].frame.bpose, H_s1c1)
        H_b0g = Hg.inv(self._shapes[0].frame.body.pose)
        H_b1g = Hg.inv(self._shapes[1].frame.body.pose)
        self._frames[0]._bpose = dot(H_b0g, H_gc0) #TODO: use a set method?
        self._frames[1]._bpose = dot(H_b1g, H_gc1)
        self._is_active = (sdist < self._proximity)
        self._sdist = sdist

    def is_active(self):
        return self._is_active


class SoftFingerContact(PointContact):
    r"""This class implements a "soft-finger" point contact constraint. 

    Let's consider two convex objects (shapes) and attach a frame to each of
    them in such a way that:
        - their origin are chosen on the object surface, where the 
          (signed) distance between the objects is minimal,
        - the two frames are aligned,
        - the normal to the object's surface is along their z-axis.

    If we denote 0 and 1 the two frames, their coordinate 
    change matrix has the following form:
    
    .. math::

        \Hg[0]_1 &=
        \begin{bmatrix}
        1 & 0 & 0 & 0 \\
        0 & 1 & 0 & 0 \\
        0 & 0 & 1 & d \\
        0 & 0 & 0 & 1
        \end{bmatrix}
    
    where `d` is the signed distance between the objects. 
    If `d = 0`, the objects are in contact, if `d < 0` they 
    penetrate each other.

    We consider here point contacts with a Coulomb friction model,
    extended to involve moment resisting to torsion, as described in 
    Liu2003_ and Trinkle2001_ and often denoted as "soft finger
    contact". The contact wrench can be decomposed as:

    .. math::
        \wrench[0]_{0/1} &=
        \begin{bmatrix}
        0 \\ 0 \\ m_z \\ f_x \\ f_y \\ f_z
        \end{bmatrix}

    The Signorini law (or complementarity condition) imposes conditions on
    the normal force (aka. pressure force):

    .. math::
        f_z &\ge 0 \\
        d &\ge 0 \\
        d \cdot f_z &=0

    which are often sumarized as `0 \le d \perp f_z \ge 0`.

    .. todo:: 
        check the contact rupture condition is good. (Duindam, in his phd, 
        chose a very specific one).
    
    Additionnaly, the Coulomb law of friction with the elliptic friction 
    model states that the contact is in static friction mode and only 
    rolling motion occurs if `d = 0` and:

    .. math::
        \frac{m_z^2}{e_p^2} + \frac{f_x^2}{e_x^2} + \frac{f_y^2}{e_y^2}
        &\le \mu^2 \cdot f_z^2

    where `e_p`, `e_x`, `e_y` are normalization coefficents
    and `\mu` is the coefficient of friction. 
    In such a case, the contact relative twist has the form:
    
    .. math::
        \twist[0]_{1/0} &=
        \begin{bmatrix}
        \omega_x \\ \omega_y \\ 0 \\ 0 \\ 0 \\ 0
        \end{bmatrix}

    Otherwise, the friction is said to be dynamic, and additionnal 
    sliding and pivoting motions occur:

    .. math::
        \twist[0]_{1/0} &=
        \begin{bmatrix}
        \omega_x \\ \omega_y \\ \omega_z \\ v_x \\ v_y \\ 0
        \end{bmatrix}

    with:

    .. math::
        \begin{bmatrix}
        \omega_z \\ v_x \\ v_y
        \end{bmatrix} = s
        \begin{bmatrix}
        \frac{m_z}{e_p^2} \\ \frac{f_x}{e_x^2} \\ \frac{f_y}{e_y^2}
        \end{bmatrix}
        \text{ and }
        s = \frac{
            \sqrt{e_p^2 \cdot \omega_z + e_x^2 \cdot v_x + e_y^2 \cdot v_y}}{
            \mu \cdot f_z}

    and

    .. math::
        \frac{m_z^2}{e_p^2} + \frac{f_x^2}{e_x^2} + \frac{f_y^2}{e_y^2}
        &= \mu^2 \cdot f_z^2

    

    References:

    .. _Liu2003:
        T. Liu and M. Y. Wang, 
        “Computation of three dimensional rigid body dynamics of 
        multiple contacts using time-stepping and Gauss-Seidel 
        method”, 
        IEEE Transaction on Automation Science and Engineering, 
        submitted, November 2003

    .. _Trinkle2001:
        J.C. Trinkle, J. Tzitzoutis, and J.S. Pang, 
        "Dynamic Multi-Rigid-Body Systems with Concurrent 
        Distributed Contacts: Theory and Examples", 
        Philosophical Trans. on Mathematical, Physical, 
        and Engineering Sciences, Series A, 359(1789):2575-2593, 
        December, 2001

    """
    def __init__(self, shapes, friction_coeff, collision_solver=None,
                 proximity=0.02, name=None):
        self._mu = friction_coeff
        PointContact.__init__(self, shapes, collision_solver, proximity)
        NamedObject.__init__(self, name)
        self._force = zeros(4)
        self._eps = array((1., 1., 1.))

    @property
    def ndol(self):
        return 4

    @property
    def jacobian(self):
        H_01 = dot(Hg.inv(self._frames[0].pose), self._frames[1].pose)
        return (dot(Hg.adjoint(H_01)[2:6,:], self._frames[1].jacobian)
                -self._frames[0].jacobian[2:6,:])


    def solve(self, vel, admittance, dt):
        r"""

        We map:
        - ``vel``: `v^*`
        - ``admittance``: `Y`
        - ``dt``: `dt`
        - ``dforce`` : `\Delta f`


        Let's define the constraint velocity
 
        .. math::
            v =  
            \begin{bmatrix}
                \omega_z \\ v_x \\ v_y \\ v_z
            \end{bmatrix}
            = S \; \twist[0]_{1/0}
        
        with
 
        .. math::
            S &= 
            \begin{bmatrix}
                0 & 0 & 1 & 0 & 0 & 0\\
                0 & 0 & 0 & 1 & 0 & 0\\
                0 & 0 & 0 & 0 & 1 & 0\\
                0 & 0 & 0 & 0 & 0 & 1
            \end{bmatrix} 
 
        and the constraint force
 
        .. math::
            f = 
            \begin{bmatrix}
                m_z \\ f_x \\ f_y \\ f_z
            \end{bmatrix} 
 
        so that  
        
        .. math::
            \wrench[0]_{0/1} = S^T \; f
 
        at the `k`-iest iteration of the Gauss-Seidel algorithm, we have
 
        .. math::
            f^k(t) &= f^{k-1}(t) + \Delta f \\
            v^k(t+dt) &= v^*(t+dt) + Y \; \Delta f \\
            d^k(t+dt) &= d(t) + dt \cdot
            \begin{bmatrix}
                0 & 0 & 0 & 1
            \end{bmatrix}
            \left( v^*(t+dt) + Y \; \Delta f \right)
 
        for static friction we'll need 
 
        .. math::
            d^k(t+dt) &= 0
        
        and
        
        .. math::
            
            \begin{bmatrix}
                1 & 0 & 0 & 0 \\
                0 & 1 & 0 & 0 \\
                0 & 0 & 1 & 0
            \end{bmatrix}
            v^k(t+dt)
            &=
            \begin{bmatrix}
                0 \\ 0 \\ 0
            \end{bmatrix}
 
        which leads to
 
        .. math::
            \Delta f &= -Y^{-1} 
            \left( 
            v^*(t+dt) + 
            \begin{bmatrix}
                0 \\ 0 \\ 0 \\ \frac{d(t)}{dt}
            \end{bmatrix}
            \right)
 
        for dynamic friction, the condition on sliding velocity and contact
        persistance lead to
 
        .. math::
            v^k(t+dt)
            &=
            s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            (f^{k-1}+\Delta f)
            +
            \begin{bmatrix}
            0 \\ 0 \\ 0 \\ -\frac{d(t)}{dt}
            \end{bmatrix} \\
            \Leftrightarrow 
            v^*(t+dt) + Y \Delta f
            &=
            s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            \left( f^{k-1}+\Delta f \right)
            +
            \begin{bmatrix}
            0 \\ 0 \\ 0 \\ -\frac{d(t)}{dt}
            \end{bmatrix}

        Finally, we get 

        .. math::
            0 &=
            \alpha + \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            \right)
            \left( f^{k-1}+\Delta f \right)

        and 
            
        .. math::
            \left( f^{k-1}+\Delta f \right)
            &=
            - \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            \right)^{-1} \alpha 
 
        while denoting
 
        .. math::
            \alpha &=
            v^*(t+dt) - 
            \begin{bmatrix}
            0 \\ 0 \\ 0 \\ -\frac{d(t)}{dt}
            \end{bmatrix} - Y f^{k-1}
 
        Note that `s` is still unknown. However,
 
        .. math::
            \frac{m_z^2}{e_p^2} + \frac{f_x^2}{e_x^2} + \frac{f_y^2}{e_y^2}
            &= \mu^2 \cdot f_z^2
 
        can be rewritten
 
        .. math::
 
            \left( f^{k-1}+\Delta f \right)^T
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & -\mu^2
            \end{bmatrix} 
            \left( f^{k-1}+\Delta f \right)
            &=0
 
        so that if we can find `s` solution of
 
        .. math::
            \alpha^T
            \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            \right)^{-T}
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & -\mu^2
            \end{bmatrix} 
            \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix} 
            \right)^{-1} \alpha 
            &=0 \text{,}
 
        then we're done.

        Let's decompose `Y`, using numpy notations, noticing that it is
        symmetric:

        .. math::
            Y &= 
            \begin{bmatrix}
            Y_{0:3,0:3} & Y_{0:3,3} \\
            Y_{3,0:3}   & Y_{3,3}
            \end{bmatrix}
            = 
            \begin{bmatrix}
            Y_t   & Y_c \\
            Y_c^T & y_n
            \end{bmatrix}

        It can then be block-diagonalized:

        .. math::
            \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix}
            \right)
            &=
            \begin{bmatrix}
            Y_t - s
                \begin{bmatrix}
                \frac{1}{e_p^2} & 0 & 0 \\
                0 & \frac{1}{e_x^2} & 0 \\
                0 & 0 & \frac{1}{e_y^2}
                \end{bmatrix} 
            & Y_c \\
            Y_c^T & y_n
            \end{bmatrix}
            \\ 
            &=
            \begin{bmatrix}
            I_{3 \times 3} & Y_c/y_n \\
            0_{1 \times 3} & 1
            \end{bmatrix}
            \begin{bmatrix}
            Y_t - Y_c\;Y_c^T/y_n - s
                \begin{bmatrix}
                \frac{1}{e_p^2} & 0 & 0 \\
                0 & \frac{1}{e_x^2} & 0 \\
                0 & 0 & \frac{1}{e_y^2}
                \end{bmatrix} 
                           & 0_{3 \times 1} \\
            0_{1 \times 3} & y_n
            \end{bmatrix}
            \begin{bmatrix}
            I_{3 \times 3} &  0_{3 \times 1} \\
            Y_c^T/y_n      & 1
            \end{bmatrix}
            
        and its inverse

        .. math::
            \left(
            Y - s 
            \begin{bmatrix}
            \frac{1}{e_p^2} & 0 & 0 & 0 \\
            0 & \frac{1}{e_x^2} & 0 & 0 \\
            0 & 0 & \frac{1}{e_y^2} & 0 \\
            0 & 0 & 0 & 0
            \end{bmatrix}
            \right)^{-1}
            &=
            \begin{bmatrix}
            I_{3 \times 3}  &  0_{3 \times 1} \\
            -Y_c^T/y_n      & 1
            \end{bmatrix}
            \begin{bmatrix}
            \left(Y_t - Y_c\;Y_c^T/y_n - s
                \begin{bmatrix}
                \frac{1}{e_p^2} & 0 & 0 \\
                0 & \frac{1}{e_x^2} & 0 \\
                0 & 0 & \frac{1}{e_y^2}
                \end{bmatrix}\right)^{-1}
                           & 0_{3 \times 1} \\
            0_{1 \times 3} & 1
            \end{bmatrix}
            \begin{bmatrix}
            I_{3 \times 3} & -Y_c/y_n \\
            0_{1 \times 3} & y_n
            \end{bmatrix}
        
        injecting this last expression into, it can be shown that `s` is
        an eigen value of the `B` matrix

        .. math::
            B &=
            \begin{bmatrix}
                e_p^2 & 0 & 0 & 0 & 0 & 0 \\
                0 & e_x^2 & 0 & 0 & 0 & 0 \\
                0 & 0 & e_y^2 & 0 & 0 & 0 \\
                0 & 0 & 0 & e_p^2 & 0 & 0 \\
                0 & 0 & 0 & 0 & e_x^2 & 0 \\
                0 & 0 & 0 & 0 & 0 & e_y^2
            \end{bmatrix}
            \begin{bmatrix}
                Y_t - Y_c\;Y_c^T/y_n + 2 \frac{\beta b^T}{a}
                    & -\frac{\beta \beta^T}{a^2} \\
                b b^T -
                \begin{bmatrix}
                    \frac{1}{e_p^2} & 0 & 0 \\
                    0 & \frac{1}{e_x^2} & 0 \\
                    0 & 0 & \frac{1}{e_y^2}
                \end{bmatrix}
                    & Y_t - Y_c\;Y_c^T/y_n
            \end{bmatrix}

        where the following values has been introduced

        .. math::
            \beta &= 
            \begin{bmatrix}
                I_{3 \times 3} & -Y_c/y_n \\
                \end{bmatrix} \alpha \\
            a &= \mu y_n \alpha_3 \\
            b &= \frac{\mu}{y_n} Y_c

        .. note::
            the full math is available as a pdf, within this document, `\beta`
            is denoted `\hat{\alpha}`.

        """
        if self._sdist + dt*vel[3]>0:
            # if there is no contact, the contact force should be 0
            dforce = -self._force
            self._force[:] = 0.
            return dforce

        else:
            # if there is contact, the normal velocity is given by the 
            # restitution model (here, zero for strictly inelastic 
            # contact). 
            # The tangent velocity is given by the Coulomb
            # friction model. 
            
            # First, try with static friction: zero tangent velocity
            dforce = solve(-admittance, 
                           hstack((vel[0:3], vel[3]+self._sdist/dt)))
            force = self._force + dforce

            if sum((force[0:3]/self._eps)**2) <= (force[3]/self._mu)**2:
                # the elliptic dry friction law is respected.
                self._force = force
                return dforce
            else:
                # the elliptic dry friction law is not respected.
                # We have to look for a solution with sliding
                #
                #E2 = c.misc.E*c.misc.E;
                #F = E2 - diag([0,0,c.misc.mu^2,0]);
                #k = solve_sliding(alpha,Y,F);
                #c.Gamma=-(Y+k*E2)\alpha;

                alpha = vel - dot(admittance, self._force)
                alpha[3] += self._sdist/dt
                Y_c = admittance[0:3,3]
                y_n = admittance[3,3]
                Y_t = admittance[0:3,0:3] - dot(Y_c, Y_c.T)/y_n
                beta = alpha[0:3] - alpha[3]/y_n*Y_c
                a = self._mu * y_n * alpha[3]
                b = self._mu/y_n * Y_c
                B = zeros((6,6))
                E = diag(self._eps**2)
                B[3:6,3:6] = dot(E, Y_t - dot(Y_c, Y_c.T)/y_n)
                B[0:3,0:3] = dot(E, B[3:6,3:6] + 2/a*dot(beta,b.T))
                B[0:3,3:6] = dot(E ,dot(beta,beta.T)/(a**2))
                B[3:6,0:3] = dot(b,b.T)- dot(E, diag(self._eps**-2))

                # s is the real part of the eigenvalue with the smallest 
                # imaginary par within those with a positive real part
                S = eigvals(B)
                S = S[S.real >= 0]
                ind = S.imag.argsort()
                s = S[ind[0]].real
                s = min(s, 1e10) #TODO: throw exception when s>=1e10

                prev_force = self._force.copy()
                A = admittance.copy()
                A[0:3,0:3] -= s*diag(self._eps**-2)
                self._force = solve(A,-alpha)
                dforce = self._force - prev_force
                return dforce
        
