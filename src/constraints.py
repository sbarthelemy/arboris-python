# coding=utf-8

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from abc import ABCMeta, abstractmethod
from numpy import zeros, eye
from numpy.linalg import solve

class BodyConstraint(object):
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
    between two bodies (more precisely, between two frames rigidly fixed to two
    distinct bodies).

    Let's denote 0 and 1 these two frames. The constraint can be expressed as a
    condition on their relative pose, requiring  than the ball and socket
    centers be co-located at the same:

    .. math::
        
        f(\Hg[0]_1) &= 0  
        \Leftrightarrow 
        \Hg[0]_1 = 
        \begin{bmatrix}
            \pre[0]R_1 & 0\\
            0            & 1
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

   The algorithm has two phases. First, a prediction of the future constraints states is done, assuming constraints forces won't change from the last time step. We'll then have :math:`\Hg[0]_1(0)`, :math:`\twist[0]_{1/0}(0)` and :math:`f(0)`.
   Secondly, adjustement of constraint forces :math:`\Delta f` are computed. As
   the several constraints may interfere one with each other, they are adjusted
   iteratively until the adjustement forces are small enough to be neglected.
   We'll note :math:`\Delta f` the variation of constraint force from the
   prediction and :math:`\Delta v` the variation of contraint velocity due to
   all the constraints forces adjustements. At each iteration, we'll update
   :math:`\Delta f` by :math:`\delta f` such that afterward

    
    .. math::
        f
        &= f(0) + \Delta f + \delta f  \\
        \twist[0]_{1/0} 
        &= \twist[0]_{1/0}(0) + S^T \; (\Delta v + Y \delta f ) \\
        \Hg[0]_{1/0} 
        &= \exp( dt \; S^T \; (\Delta v + Y \; \delta f ) ) \; \Hg[0]_{1/0}(0)
        \\
        \pre[0]p_1 
        &= \pre[0]p_1(0) + dt \; (\Delta v +  Y \; \delta f) 
    where :math:`Y` is the constraint admittance matrix. Before the next
    iteration, the constraint force adjustment is updated: 
    :math:`\Delta f := \Delta f + \delta f`.

    W_10: constraint wrench exerted by 0 on 1, expressed in 1. Thus, the power
    dissipated in the constraint is V_10 * W_01

    p_01 = p_01_0 + dt*dvel_10
        p_01_next == p_01_0 + dt*dvel + dt*Y*ddforce
        we want p_01_next == 0 so we choose
        ddforce = -Y^{-1} *(p_0/dt + dvel) 

    its linearization gives
    """
   
    def __init__(self):
        self._force0 = zeros(3)
        self._pos0 = zeros(3)
        self._dforce = zeros(3)

    def ndol(self):
        return 3

    def is_active(self):
        return True

    def force(self):
        return self._force0 + self._dforce

    def predict(self):
        self._force0 += self._dforce
        self._dforce[:] = 0.
        self._pos0 = None

    def init(self):
        pass
        #TODO: update self._pos0

    def jacobian(self):
        pass
        #TODO: add adjoint
        return (self._frames[0].jacobian()[3:6,:]
                -self._frames[1].jacobian()[3:6,:])

    def solve(self, dvel, admittance, dt):
        r"""

        from this equation
        
        .. math::
            \pre[0]p_1 
             &= \pre[0]p_1(0) + dt \; (\Delta v +  Y \; \delta f) 

        given that we want :math:`\pre[0]p_1 = 0`, we return 

        .. math::
            \delta f = -Y^{-1} \; 
            \left(
                \frac{\pre[0]p_1(0)}{dt}) + \Delta v 
            \right)

        and update the constraint force adjustment :math:`\Delta f` 

        """
        ddforce = solve(-admittance, dvel+self._pos0/dt)
        self._dforce += ddforce
        return ddforce

