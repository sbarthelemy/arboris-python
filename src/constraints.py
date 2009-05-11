# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

"""

.. todo:
    merge Constraint and BodyConstraint classes? This will depend on the joint
    stops constraints implementation

"""

from abc import ABCMeta, abstractmethod
from numpy import array, zeros, eye, dot
from numpy.linalg import solve
import homogeneousmatrix as Hg
from misc import NamedObject


class Constraint(NamedObject):

    def __init__(self, name=None):
        NamedObject.__init__(self, name)
    
    @abstractmethod
    def gforce(self):
        pass



class BodyConstraint(Constraint):
    __metaclass__ = ABCMeta

    def gforce(self):
        return dot(self.jacobian().T, self._force)

    @abstractmethod
    def jacobian(self):
        pass

    @abstractmethod
    def ndol(self):
        """
        number of degree of "liaison" (in french: *nombre de degrés de liaison*)
        This is equal to 6-ndof
        """
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def is_active(self):
        pass

    @abstractmethod
    def solve(self, dforce, dvel, admittance, dt):
        pass

class BallAndSocketConstraint(BodyConstraint):
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

    Let's define the constraint velocity :math:`v` and the constraint force
    :math:`f` as

    .. math::

        v &= S \; \twist[0]_{1/0} = \pre[0]{\dot{p}}_1 \\
        \wrench[0]_{0/1} &= S^T \; f  \\
        S &= 
        \begin{bmatrix}
        0 & 0 & 0 & 1 & 0 & 0\\
        0 & 0 & 0 & 0 & 1 & 0\\
        0 & 0 & 0 & 0 & 0 & 1
        \end{bmatrix} 

   The constraint jacobian is then given by :math:`( \Ad[0]_1 \; \pre[1]J_{1/g} - \pre[0]J_{0/g} )`

   In order to solve the constraint, an adjustement :math:`\Delta f` of the
   constraint force is computed by the ``solve`` method. 
   This method is given as argument the current estimation of constraint
   velocity, which corresponds to :math:`\Delta f = 0` and takes into account
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
   
    def __init__(self, name=None):
        self._force = zeros(3)
        self._pos0 = None
        NamedObject.__init__(self, name)

    def ndol(self):
        return 3

    def update(self):
        r"""
        Compute the predicted relative position error between the socket and
        ball centers, :math:`\pre[0]p_1(t)` and save it in ``self._pos0``.

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

        given that we want :math:`\pre[0]p_1(t+dt) = 0`, we return 

        .. math::
            \Delta f = -Y^{-1} \; 
            \left(
                \frac{\pre[0]p_1(t)}{dt} + v^* 
            \right)

        and update the constraint force adjustment :math:`\Delta f` 

        The function arguments are

        - ``vel``: :math:`v^*`
        - ``admittance``: :math:`Y`
        - ``dt``: :math:`dt`

        Tests:

        >>> c = BallAndSocketConstraint()
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

    def gforce(self):
        return dot(self.jacobian().T, self._force)


proximity = 0.01

class PointContact(Constraint):
    r"""
    
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

    def __init__(self, shapes, collision_solver):
        r"""
        .. todo:
            find the good ``collision_solver`` automatically according
            to the ``shapes`` pair
        """
        self._shapes = shapes
        self._frames = (shapes[0].frame.body.newframe(),
                        shapes[1].frame.body.newframe())
        self._collision_solver = collision_solver


    def update(self):
        r"""
        This method calls the collision solver and updates the constraint
        status and the contact frames poses accordingly.
        The two frames have the same orientation, with the contact normal along
        the :math:`z`-axis
        """
        (sdist, H_s0c0, H_s1c1) = self._collision_solver(shapes)
        (sdist, H_gc0, H_gc1) = self._collision_solver(shapes)
        #self._frames[0].bpose = dot(self._shapes[0].frame.bpose, H_s0c0)
        #self._frames[1].bpose = dot(self._shapes[1].frame.bpose, H_s1c1)
        H_b0g = Hg.inv(self._shapes[0].frame.body.bpose)
        H_b1g = Hg.inv(self._shapes[1].frame.body.bpose)
        self._frames[0].bpose = dot(H_b0g, H_gc0)
        self._frames[1].bpose = dot(H_b1g, H_gc1)
        self._is_active = (sdist < proximity)
        self._sdist = sdist


    def is_active(self):
        return self._is_active


class SoftFingerContact(PointContact):
    r"""
   
    This class implements a "soft-finger" point contact constraint. 

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
    
    where :math:`d` is the signed distance between the objects. 
    If :math:`d = 0`, the objects are in contact, if :math:`d < 0` they 
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

    which are often sumarized as: :math:`0 \le d \perp f_z \ge 0`.

    .. todo:: 
        check the contact rupture condition is good. (Duindam, in his phd, 
        chose a very specific one).
    
    Additionnaly, the Coulomb law of friction with the elliptic friction 
    model states that the contact is in static friction mode and only 
    rolling motion occurs if :math:`d = 0` and:

    .. math::
        \frac{m_z^2}{e_p^2} + \frac{f_x^2}{e_x^2} + \frac{f_y^2}{e_y^2}
        &\le \mu^2 \cdot f_z^2

    where :math:`e_p`, :math:`e_x`, :math:`e_y` are normalization coefficents
    and :math:`\mu` is the coefficient of friction. 
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
        \end{bmatrix}
        = k
        \begin{bmatrix}
        \frac{m_z}{e_p^2} \\ \frac{f_x}{e_x^2} \\ \frac{f_y}{e_y^2}
        \end{bmatrix} 
        \text{   and   } 
        k = \frac{
            \sqrt{e_p^2 \cdot \omega_z + e_x^2 \cdot v_x + e_y^2 \cdot v_y}}{
            \mu \cdot f_z}

    

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
    def __init__(self, shapes, friction_coeff, collision_solver, name=None):
        self._friction_coeff = friction_coeff
        PointContact.__init__(self, shapes, collision_solver)
        NamedObject.__init__(self, name)

    def jacobian(self):
        H_01 = dot(Hg.inv(self._frames[0].wpose()), self._frames[1].wpose())
        return (dot(Hg.adjoint(H_01)[2:6,:], self._frames[1].jacobian())
                -self._frames[0].jacobian()[2:6,:])


    def solve(self, vel, admittance, dt):
        r"""

    Let's define the constraint velocity

    .. math::
        v =  S \; \twist[0]_{1/0} =
        \begin{bmatrix}
            \omega_z \\ v_x \\ v_y \\ v_z
        \end{bmatrix}
    
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

    at the :math:`k`-iest iteration of the Gauss-Seidel algorithm, we have

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

    for dynamic friction, 

    .. math::
        v^k(t+dt)
        &=
        k 
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
        v^*(t+dt) + Y \Delta f
        &=
        k 
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
        \end{bmatrix}

    .. math::
        0 &= \Delta v  \left( I -k Y^{-1} E^2 \right) f \\
        \Delta v &= Y^{-1} \left( 
        \begin{bmatrix}
        w_z \\ v_x \\ v_y \\ d_0
        \end{bmatrix}\right)

        - ``vel``: :math:`v^*`
        - ``admittance``: :math:`Y`
        - ``dt``: :math:`dt`
        - ``dforce`` : :math:`\Delta f`
        """
        if self._pos0[3] + dt*vel(3)>0:
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
            b = vstack(vel[0:3], self._pos0[3]/dt)
            dforce = solve(admittance, 
                           hstack((vel[0:3], vel[3]+self._pos0[3]/dt)))
            force = self._force + dforce

            if sum((force[0:3]/self.eps)**2) <= (force[3]/self._mu)**2:
                # the elliptic dry friction law is respected.
                self._force = force
                return dforce
            else:
                # the elliptic dry friction law is not respected.
                # We have to look for a solution with sliding

                #alpha = -Y*c.Gamma;
                #E2 = c.misc.E*c.misc.E;
                #F = E2 - diag([0,0,c.misc.mu^2,0]);
                #k = solve_sliding(alpha,Y,F);
                #c.Gamma=-(Y+k*E2)\alpha;
                self._force = force
                return dforce

if __name__ == "__main__":
    import doctest
    doctest.testmod()
