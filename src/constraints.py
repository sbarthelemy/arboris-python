# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from abc import ABCMeta, abstractmethod
from numpy import array, zeros, eye, dot
from numpy.linalg import solve
import homogeneousmatrix as Hg
from misc import NamedObject

class Constraint(NamedObject):

    def __init__(self, name=None):
        NamedObject.__init__(self, name)

class BodyConstraint(Constraint):
    __metaclass__ = ABCMeta

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
    def is_active():
        pass

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def solve(self, dforce, dvel, admittance, dt):
        """
        cforce = self.cforce0 + dcforce
        cvel = self.cvel0 + dcvel
        cQ = self.cQ0 * exp(dt * dcvel)
        cforce_next = self.cforce0 + dcforce + ddcforce
        cvel = self.cvel0 + dcvel 
        cvel_next = self.cvel0 + dcvel + admittance*(ddforce)
        cQ_next = self.cQ0 * exp(dt * dcvel*admittance*(ddforce) )
        """
        pass

class BallAndSocketConstraint(BodyConstraint):
    r"""
    This class describes and solves a ball and socket kinematic constraint
    between two frames which are rigidly fixed to two distinct bodies.

    Let's denote 0 and 1 these two frames. The constraint can be expressed as a
    condition on their relative pose, requiring  than the ball and socket
    centers be co-located at the same:

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
        \wrench[0]_{1/0} &= S^T \; f  \\
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
        &= v(t+dt) + Y(t) \Delta f  \\
        \Hg[0]_{1/0}^k(t+dt)
        &= \exp( dt \; S^T \; ( v(t+dt) + Y(t) \; \Delta f ) ) \;
        \Hg[0]_{1/0}(t)
        \\
        \pre[0]p_1^k(t+dt)
        &= \pre[0]p_1(t) + dt \; (v(t+dt) +  Y(t) \; \Delta f) 

    """
   
    def __init__(self, name=None):
        self._force = zeros(3)
        self._pos0 = None
        Constraint.__init__(self, name)

    def ndol(self):
        return 3

    def is_active(self):
        return True

    def init(self):
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
        H_01 = dot(Hg.inv(self._frames[0].wpose()), self._frames[1].wpose())
        self._pos0 = H_01[0:3,3]

    def jacobian(self):
        H_01 = dot(Hg.inv(self._frames[0].wpose()), self._frames[1].wpose())
        return (dot(Hg.adjoint(H_01)[3:6,:], self._frames[1].jacobian())
                -self._frames[0].jacobian()[3:6,:])

    def solve(self, vel, admittance, dt):
        r"""

        from this equation
        
        .. math::
            \pre[0]p_1(t+dt) 
             &= \pre[0]p_1(t) + dt \; ( v +  Y \; \Delta f) 

        given that we want :math:`\pre[0]p_1(t+dt) = 0`, we return 

        .. math::
            \Delta f = -Y^{-1} \; 
            \left(
                \frac{\pre[0]p_1(t)}{dt} + v 
            \right)

        and update the constraint force adjustment :math:`\Delta f` 

        The function arguments are

        - ``vel``: :math:`\Delta v`
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

if __name__ == "__main__":
    import doctest
    doctest.testmod()
