# coding=utf-8
"""
...

A world (instance of the :class:`World` class) consists of bodies (instances of the :class:`Body` class) interlinked by joints (instances of :class:`Joint` subclasses). Joints serve two purposes in arboris: 
    
- restrict the relative motion between bodies (for instance a hinge joint only allows for rotations around its axis) 
- and parametrize the bodies positions (for instance a single angle is enough to parametrize the relative position of two bodies constrained by a hinge joint).

A world forms an oriented tree whose nodes are the bodies and edges are the
joints, so that the state (pose and velocity) of each body can be computed from
the state of its parent joints (the joints on the path from the body to the root
of the tree (which is body called "ground")).

One or more frames (instances of the :class:`Frame` class) can be associated to
bodies and serve as anchor points to the joints

TODO: 

- put object names in a dict. replace bodies by an iterator
- import human36 from matlab-arboris
- add unit tests for human36
- add support for controllers and integration
- add __repr__ or unicode methods
- split world description and its state+matrices ? 
- add support for visu
- add support for explicit joints
- add support for contacts and joint limits

IDEAS:

- add dpose() to RigidMotion
- add support for coupled joints
- add support for non-holonomic joints

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy import ix_, array, zeros, ones, eye, dot
import numpy
import homogeneousmatrix as Hg
import homogeneousmatrix
import twistvector as T
import adjointmatrix
from abc import ABCMeta, abstractmethod
from math import pi
from controllers import Controller
from joints import Joint
from constraints import Constraint
from misc import NamedObject

class World(NamedObject):

    """

    TODO: provide ability to merge worlds or subtrees
    
    >>> from  worldfactory import triplehinge
    >>> w = triplehinge()
    >>> w.update_geometric()
    >>> w.update_dynamic()
    """

    def __init__(self, name=None):
        NamedObject.__init__(self, name)
        self.ground = Body(u"ground")
        self.bodies = [self.ground] 
        self.joints = []
        self._controllers = []
        self._constraints = []
        self._collisions = []
        self._ndof = 0
        self._gvel = array([])
        self._mass =  array([])# updated by self.update_dynamic()
        self._gforce =  array([])
        self._viscosity = array([]) # updated by self.update_dynamic()
        self._controller_viscosity = array([]) # updated by self.update_dynamic()
        self._nleffects = array([]) # updated by self.update_dynamic()


    @property
    def mass(self):
        return self._mass

    @property
    def viscosity(self):
        return self._viscosity

    @property
    def nleffects(self):
        return self._nleffects


    def ndof(self):
        return self._ndof


    def add_joint(self, joint, frames):
        """Add a joint and its new-attached body to the world.

        :arguments:
            joint
                the joint that will be added

        Examples:
        >>> from  worldfactory import triplehinge
        >>> w = triplehinge()
        >>> w.joints[0].gvel[0] = 1.
        >>> w._gvel
        array([ 1.,  0.,  0.])
        >>> w._gvel[1] = 2.
        >>> w.joints[1].gvel
        array([ 2.])

        """
        if not(isinstance(joint, Joint)):
            raise ValueError("{0} is not an instance of Joint".format(
                joint))
        if not(isinstance(frames[0], Frame)):
            raise ValueError("{0} is not an instance of Frame".format(
                frames[0]))
        if not(isinstance(frames[1], Frame)):
            raise ValueError("{0} is not an instance of Frame".format(
                frames[1]))

        # check the reference/base/parent frame for the joint is already 
        # in world
        frame0_is_in_world = False
        for b in self.bodies:
            if frames[0].body is b:
               frame0_is_in_world = True
        if not(frame0_is_in_world):
            raise ValueError("The reference/base/parent frame is attached to a body that is not in world")
        
        new_body = frames[1].body
        # check the new/local/child frame is not already in world
        for b in self.bodies:
            if new_body is b:
               raise ValueError("The new/local/child  frame is attached to a body that is already in world")
        if (new_body.parentjoint is not None):
            raise ValueError("The new/local/child frame is attached to a body that already has a parent joint")
        if len(new_body.childrenjoints) > 0:
            raise ValueError("The new/local/child frame is attached to a body that already has a children joints")
        
        # extend the world generalized velocities
        old_ndof = self._ndof
        self._ndof = self._ndof + joint.ndof()
        
        # add the joint and the moving frame to the world
        joint._frames = frames
        joint._dof = slice(old_ndof, self._ndof)
        self.joints.append(joint)
        joint._frames[0].body.childrenjoints.append(joint)
        self.bodies.append(new_body)
        new_body.parentjoint = joint

        self._gvel = zeros(self._ndof)
        self._mass = zeros((self._ndof,self._ndof))
        self._nleffects =  zeros((self._ndof,self._ndof))
        self._viscosity = zeros((self._ndof,self._ndof))
        self._controller_viscosity = zeros((self._ndof,self._ndof))
        self._gforce = zeros(self._ndof)

        for j in self.joints:
            self._gvel[j._dof] = j.gvel[:]
            j.gvel = self._gvel[j._dof]


    def add_jointcontroller(self, controller, joints=None):
        """
        Add a joint controller to the world

        Example:

        >>> from worldfactory import triplehinge
        >>> w = triplehinge()
        >>> from controllers import ProportionalDerivativeController
        >>> c0 = ProportionalDerivativeController(w.joints[1:3], name = 'my controller')
        >>> w.add_jointcontroller(c0, w.joints[1:3])
        >>> c1 = ProportionalDerivativeController(w.joints[0:1])
        >>> w.add_jointcontroller(c1, w.joints[0:1])
        
        """
        for c in self._controllers:
            if c is controller:
                raise ValueError("the controller is already in the world")

        if joints is None:
            controller._dof = range(self.ndof())
        else:
            controller._dof = []
            for j in joints:
                controller._dof.extend(range(j._dof.start, j._dof.stop))
        
        self._controllers.append(controller)


    def add_constraint(self, constraint, frames):
        constraint._frames = frames #TODO: do some consistency checks
        self._constraints.append(constraint)
        

    def update_geometric(self):
        """
        Compute the forward geometric model. 
        
        This will recursively update each body pose attribute.
        """
        self.bodies[0].geometric(eye(4))


    def update_kinematic(self):
        """
        Compute the forward geometric and kinematic models. 
        
        This will recursively update all each body pose and jacobian
        attributes.
        """
        self.bodies[0].kinematic(eye(4),zeros((6,self._ndof)))


    def update_dynamic(self):
        """
        Compute the forward geometric, kinematic and dynamic models. 
        
        This will recursively update 
        
        - each body pose, jacobian, djacobian, twist and nleffects 
          attributes 
        - the world mass, viscosity and nleffects attributes

        >>> import worldfactory
        >>> w = worldfactory.triplehinge()
        >>> w.joints[0].gpos[0]=0.5
        >>> w.joints[0].gvel[0]=2.5
        >>> w.joints[1].gpos[0]=1.0
        >>> w.joints[1].gvel[0]=-1.0
        >>> w.joints[2].gpos[0]=2.0/3.0
        >>> w.joints[2].gvel[0]=-0.5
        >>> w.update_dynamic()
        >>> w.bodies[1].pose
        array([[ 0.87758256, -0.47942554,  0.        ,  0.        ],
               [ 0.47942554,  0.87758256,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> w.bodies[2].pose
        array([[ 0.0707372 , -0.99749499,  0.        , -0.23971277],
               [ 0.99749499,  0.0707372 ,  0.        ,  0.43879128],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> w.bodies[3].pose
        array([[-0.56122931, -0.82766035,  0.        , -0.63871076],
               [ 0.82766035, -0.56122931,  0.        ,  0.46708616],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> w.bodies[0].jacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w.bodies[1].jacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 1.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w.bodies[2].jacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 1.        ,  1.        ,  0.        ],
               [-0.27015115,  0.        ,  0.        ],
               [ 0.42073549,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> w.bodies[3].jacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 1.        ,  1.        ,  1.        ],
               [-0.26649313, -0.3143549 ,  0.        ],
               [ 0.7450519 ,  0.24734792,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> w.bodies[0].twist
        array([ 0.,  0.,  0.,  0.,  0.,  0.])
        >>> w.bodies[1].twist
        array([ 0. ,  0. ,  2.5,  0. ,  0. ,  0. ])
        >>> w.bodies[2].twist
        array([ 0.        ,  0.        ,  1.5       , -0.67537788,  1.05183873,  0.        ])
        >>> w.bodies[3].twist
        array([ 0.        ,  0.        ,  1.        , -0.35187792,  1.61528183,  0.        ])
        >>> w.viscosity
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w.bodies[1].mass
        array([[  8.35416667e-02,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   2.50000000e-01],
               [  0.00000000e+00,   4.16666667e-04,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   8.35416667e-02,
                 -2.50000000e-01,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -2.50000000e-01,
                  1.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   1.00000000e+00,   0.00000000e+00],
               [  2.50000000e-01,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
        >>> w.bodies[2].mass
        array([[  4.27733333e-02,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   1.60000000e-01],
               [  0.00000000e+00,   2.13333333e-04,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   4.27733333e-02,
                 -1.60000000e-01,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -1.60000000e-01,
                  8.00000000e-01,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   8.00000000e-01,   0.00000000e+00],
               [  1.60000000e-01,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   8.00000000e-01]])
        >>> w.bodies[3].mass
        array([[  2.67333333e-03,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   2.00000000e-02],
               [  0.00000000e+00,   1.33333333e-05,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   2.67333333e-03,
                 -2.00000000e-02,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -2.00000000e-02,
                  2.00000000e-01,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   2.00000000e-01,   0.00000000e+00],
               [  2.00000000e-02,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   2.00000000e-01]])
        >>> w.mass
        array([[ 0.55132061,  0.1538999 ,  0.0080032 ],
               [ 0.1538999 ,  0.09002086,  0.00896043],
               [ 0.0080032 ,  0.00896043,  0.00267333]])
        >>> w.bodies[0].djacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w.bodies[1].djacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w.bodies[2].djacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [-0.42073549,  0.        ,  0.        ],
               [-0.27015115,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> w.bodies[3].djacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [-0.87022993, -0.12367396,  0.        ],
               [-0.08538479, -0.15717745,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> w.bodies[0].nleffects
        array([[ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.]])
        >>> w.bodies[1].nleffects
        array([[  0.00000000e+00,  -1.04166667e-03,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  5.26041667e-02,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   6.25000000e-01,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,  -2.50000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -6.25000000e-01,
                  2.50000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        >>> w.bodies[2].nleffects
        array([[  0.00000000e+00,  -3.20000000e-04,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  1.61600000e-02,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,  -2.22261445e-18],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   2.40000000e-01,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,  -1.20000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -2.40000000e-01,
                  1.20000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        >>> w.bodies[3].nleffects
        array([[  0.00000000e+00,  -1.33333333e-05,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  6.73333333e-04,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,  -1.10961316e-18],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   2.00000000e-02,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,  -2.00000000e-01,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,  -2.00000000e-02,
                  2.00000000e-01,   0.00000000e+00,   0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        >>> w.nleffects
        array([[ 0.11838112, -0.15894538, -0.01490104],
               [ 0.27979997,  0.00247348, -0.00494696],
               [ 0.03230564,  0.00742044,  0.        ]])

        """        
        self.ground.update_dynamic(
            eye(4),
            zeros((6,self._ndof)),
            zeros((6,self._ndof)),
            zeros(6))
        
        self._mass[:] = 0.
        self._viscosity[:] = 0.
        self._nleffects[:] = 0.
        for b in self.ground.descendants():
            self._mass += dot(
                dot(b.jacobian.T, b.mass),
                b.jacobian)
            self._viscosity += dot(
                dot(b.jacobian.T, b.viscosity),
                b.jacobian)
            self._nleffects += dot(
                b.jacobian.T,
                dot(b.mass, b.djacobian) + dot(b.nleffects, b.jacobian))

    def update_controllers(self, dt):
        """

        TODO: check the two last tests results!
        >>> from worldfactory import triplehinge
        >>> w = triplehinge()
        >>> from controllers import ProportionalDerivativeController
        >>> c0 = ProportionalDerivativeController( w.joints[1:2], 2.)
        >>> w.add_jointcontroller(c0, w.joints[1:2])
        >>> w.update_dynamic()
        >>> w.update_controllers(0.001)
        >>> w._controller_viscosity
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> w._impedance
        array([[ 686.98833333,  223.44666667,   20.67333333],
               [ 223.44666667,   93.44666667,   10.67333333],
               [  20.67333333,   10.67333333,    2.67333333]])
        >>> w._admittance
        array([[ 0.00732464, -0.02030368,  0.02442014],
               [-0.02030368,  0.07595336, -0.14623345],
               [ 0.02442014, -0.14623345,  0.76905959]])

        """
        self._controller_viscosity[:] = 0.
        self._gforce[:] = 0.
        for c in self._controllers:
            c.update(dt)

            self._controller_viscosity[
                ix_(c._dof, c._dof)] += c.viscosity()
            self._gforce[c._dof] += c.gforce()
        
        self._impedance = self._mass/dt + self._viscosity - \
                self._controller_viscosity + self._nleffects
        self._admittance = numpy.linalg.inv(self._impedance)

    def update_constraints(self, dt):
        r"""
        
        Algorithm:

        In accordance with the integration scheme, we assume a first
        order model betwen generalized velocities and generalized 
        forces:

        .. math::

            \GVel(t+dt) 
            &= Y'(t) 
            \left( 
                M(t)  \GVel(t) 
                + dt \; \GForce(t)
            \right)

        where
        - the admittance matrix  :math:`Y'` takes into account a 
          first order model of the actuators,
        - the actuators generalized forces :math:`\GForce(t)` 
          are assumed to be constant during the :math:`[t , t+dt ]`
          time interval.
        
        This (constraint-free) model must be completed by constraints 
        forces :math:`\pre[c]f`, which are mapped to generalized forces
        by the constraint jacobian :math:`\pre[c]J_c^T`:

        .. math::

            \GVel(t+dt) 
            &= Y'(t) 
            \left( 
                M(t)  \GVel(t) 
                + dt \; \GForce(t)
                + \sum_{c} \; \pre[c]J_{c}^T(t) \; \pre[c]f(t)
            \right)
  
        one can also define the constraint velocity  as: 
        :math:`\pre[c]v = \pre[c]J_c \; \GVel` so that:
    
        .. math::
            
            \pre[c]v(t+dt) 
            &= \pre[c]J_c(t) \; \GVel(t+dt)\\
            &= \pre[c]J_c(t) \; Y'(t) 
            \left( 
                M(t)  \GVel(t) 
                + dt \; \GForce(t)
            \right)
            + \sum_d \;
            \pre[c]J_c(t) \; Y'(t) \; \pre[d]J_d^T(t) 
            \; \pre[d]f(t)
        
        one can define the (global) constraints velocity :math:`v`, 
        force :math:`f`, jacobian matrix :math:`J` 
        and admittance matrix :math:`Y'`:

        .. math::

            J(t) &=
            \begin{bmatrix}
                \pre[0]J_0(t)\\
                \vdots\\
                \pre[c]J_c(t)\\
                \vdots
            \end{bmatrix}\\
            v &= 
            \begin{bmatrix}
                \pre[0]v(t)\\ \vdots \\ \pre[c]v(t) \\ \vdots
            \end{bmatrix}\\
            f(t) &= 
            \begin{bmatrix}
                \pre[0]f(t)\\ \vdots \\ \pre[c]f(t) \\ \vdots
            \end{bmatrix}\\
            Y(t) &= 
            J(t) \; Y'(t) \; J(t)^T

        and get a synthetic expression:

        .. math::

            v(t+dt) &= J(t) \; Y'(t) 
            \left( M(t)  \GVel(t) + dt \; \GForce(t) \right)
            + Y(t) \; f


        This method computes the constraint forces in three steps:

        - ask each constraint for its jacobian 
        - compute :math:`J`, :math:`v`  and :math:`Y`
        - iterate over each constraint object in order to compute 
          :math:`\pre[c]f`. At each iteration the force is 
          updated by :math:`\Delta\pre[c]f`
        

        Test:
        >>> b0 = Body(mass = eye(6))
        >>> from joints import FreeJoint
        >>> j0 = FreeJoint()
        >>> w = World()
        >>> w.add_joint(j0, (w.ground.frames[0], b0.frames[0]) )
        >>> j1 = FreeJoint()
        >>> b1 = Body(mass = eye(6))
        >>> w.add_joint(j1, (b0.frames[0], b1.frames[0]) )
        >>> from controllers import WeightController
        >>> ctrl =  WeightController(w.ground)
        >>> w.add_jointcontroller(ctrl)
        >>> from constraints import BallAndSocketConstraint 
        >>> c0 = BallAndSocketConstraint()
        >>> w.add_constraint(c0, (w.ground.frames[0], b0.frames[0]) )
        >>> w.update_dynamic()
        >>> dt = 0.001
        >>> w.update_controllers(dt)
        >>> w.update_constraints(dt)
        >>> c0._force
        array([ 0.  ,  9.81,  0.  ])
        >>> w.integrate(dt)
        >>> w.update_dynamic()
        >>> b0.pose
        array([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00,
                 -3.09167026e-22],
               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]])

        """

        constraints = []
        ndol = 0
        for c in self._constraints:
            if c.is_active():
                c._dol = slice(ndol, ndol+c.ndol())
                ndol = ndol + c.ndol()
                constraints.append(c)

        jac = zeros((ndol, self._ndof))
        admittance = zeros((ndol, ndol))
        for c in constraints:
            c.init()
            jac[c._dol,:] = c.jacobian()

        vel = dot(jac, dot(self._admittance, 
                           dot(self._mass, self._gvel/dt) + self._gforce))
        admittance = dot(jac, dot(self._admittance, jac.T))

        k=0
        while k < 20: #TODO change the break condition
            k+=1 
            for c in constraints:
                dforce = c.solve(vel[c._dol], admittance[c._dol,c._dol], dt)
                vel += dot(admittance[:,c._dol], dforce)
        for c in constraints:
            self._gforce += c.gforce()


    def integrate(self, dt):
        r"""
        TODO: add support for kinematic controllers
        TODO: add support fo external wrenches
        TODO: check the last test result!
        >>> from worldfactory import triplehinge
        >>> w = triplehinge()
        >>> w.joints[1].gpos[:] = -1.
        >>> from controllers import ProportionalDerivativeController
        >>> c0 = ProportionalDerivativeController(w.joints[1:2], 1.)
        >>> w.add_jointcontroller(c0, w.joints[1:2])
        >>> w.update_dynamic()
        >>> dt = 0.001
        >>> w.update_controllers(dt)
        >>> w.integrate(dt)
        >>> w._gvel
        array([-0.00709132,  0.03355273, -0.09131555])
        """
        self._gvel[:] = dot(self._admittance, 
                            dot(self._mass, self._gvel/dt) + self._gforce)
        for j in self.joints:
            j.integrate(dt)



class Frame(NamedObject):

    def __init__(self, body, bpose=None, name=None):
        """Create a frame rigidly fixed to a body. 
        
        >>> b = Body()
        >>> f = Frame(b,Hg.rotz(3.14/3.),'Brand New Frame')
        
        The ``body`` argument must be a member of the ``Body`` class:
        >>> f = Frame(None, Hg.rotz(3.14/3.))
        Traceback (most recent call last):
            ...
        ValueError: The ``body`` argument must be an instance of the ``Boby`` class

        The ``pose`` argument must be an homogeneous matrix:
        >>> b = Body()
        >>> f = Frame(b, ones((4,4)))
        Traceback (most recent call last):
            ...
        ValueError: [[ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]] is not an homogeneous matrix

        """
        if bpose is None:
            bpose = eye(4)
        
        NamedObject.__init__(self, name)
        Hg.checkishomogeneousmatrix(bpose)
        self._bpose = bpose
        if not(isinstance(body,Body)):
            raise ValueError("The ``body`` argument must be an instance of the ``Boby`` class")
        else:
            self.body = body
    
    @property
    def bpose(self):
        return self._bpose.copy()

    def wpose(self):
        return dot(self.body.pose, self._bpose)
    
    def twist(self):
        return dot(Hg.iadjoint(self._bpose), self.body.twist)

    def jacobian(self):
        return dot(Hg.iadjoint(self._bpose), self.body.jacobian)

class Body(NamedObject):

    def __init__(self, name=None, mass=None, viscosity=None):
        if mass is None:
            mass = zeros((6,6))
        else:
            pass #TODO: check the mass matrix
        if viscosity is None:
            viscosity = zeros((6,6))
        else:
            pass #TODO: check the viscosity matrix

        NamedObject.__init__(self, name)
        # the object will have a frame, with the same name as the object itself
        self.frames = [Frame(self, eye(4), name)]
        self.parentjoint = None
        self.childrenjoints = []
        self.mass = mass
        self.viscosity = viscosity
        self._pose = None # updated by update_{geometric,kinematic,dynamic}
        self._jacobian = None # updated by update_{geometric,kinematic,dynamic}
        self._djacobian = None # updated by update_dynamic
        self._twist = None # updated by update_dynamic
        self._nleffects = None # updated by update_dynamic

    def descendants(self):
        """Iterate over all descendant bodies, with a depth-first strategy"""
        from itertools import imap
        for b in imap(lambda x: x._frames[1].body, self.childrenjoints):
            yield b
            for bb in b.descendants():
                yield bb

    def ancestors(self):
        from itertools import imap
        if self.parentjoint is not None:
            parent = self.parentjoint._frames[0].body
            yield parent
            for a in parent.ancestors():
                yield a

    @property
    def pose(self):
        return self._pose

    @property
    def jacobian(self):
        return self._jacobian

    @property
    def djacobian(self):
        return self._djacobian

    @property
    def twist(self):
        return self._twist

    @property
    def nleffects(self):
        return self._nleffects

    def reset(self):
        """
        TODO: reset everythong or remove the method
        """
        self._pose = None
        self._jacobian = None
        self._djacobian = None
        self._twist = None
        self._nleffects = None

    def newframe(self,pose,name=None):
        frame = Frame(self,pose,name)
        self.frames.append(frame)
        return frame
        
    def geometric(self,pose):
        """
        - g: ground body
        - p: parent body
        - c: child body
        - r: reference frame of the joint, rigidly fixed to the parent body
        - n: new frame of the joint, rigidly fixed to the child body
        
        so H_nc and H_pr are constant.

        H_gc = H_gp * H_pc
             = H_gp * (H_pr * H_rn * H_nc)
        """
        self._pose = pose
        H_gp = pose
        for j in self.childrenjoints:
            H_cn = j._frames[1]._bpose
            H_pr = j._frames[0]._bpose
            H_rn = j.pose()
            H_pc = dot(H_pr, dot(H_rn, Hg.inv(H_cn)))
            child_pose = dot(H_gp, H_pc)
            j._frames[1].body.geometric(child_pose)
        
    def kinematic(self,pose,jac):
        raise NotImplemented #TODO: remove the method or implement it
    
    def update_dynamic(self,pose,jac,djac,twist):
        """
        T_ab: velocity of {a} relative to {b} expressed in {a} (body twist)

        - g: ground body
        - p: parent body
        - c: child body
        - r: reference frame of the joint, rigidly fixed to the parent body
        - n: new frame of the joint, rigidly fixed to the child body
        
        so H_nc and H_pr are constant.

        H_gc = H_gp * H_pc
             = H_gp * (H_pr * H_rn * H_nc)

        T_cg = Ad_cp * T_pg + T_cp
             = Ad_cp * T_pg + Ad_cn * T_nr
             = Ad_cp * J_pg * dQ + Ad_cn * J_nr * dq
             = J_cg * dQ
        with 
        dq = [0...0 I 0...0 ] *  dQ  and
        J_cg = Ad_cp * J_pg + [ 0...0 (Ad_cn * J_nr) 0...0 ]

        dT_cg = dAd_cp * J_pg * dQ 
                + Ad_cp * dJ_pg * dQ  
                + Ad_cp * dJ_pg * ddQ 
                + Ad_cn * dJ_nr * dq
                + Ad_cn * J_nr * ddq
              = J_cg * ddQ + dJ_cg * dQ 
        so:
        dJ_cg * dQ = dAd_cp * J_pg * dQ 
                     + Ad_cp * dJ_pg * dQ  
                     + Ad_cn * dJ_nr * dq
        dJ_cg = dAd_cp * J_pg + Ad_cp * dJ_pg 
                + [ 0...0 (Ad_cn * dJ_nr) 0...0 ]
        with dAd_cp = Ad_cn * dAd_nr * Ad_rp

        """
        self._pose = pose
        self._jacobian = jac
        self._djacobian = djac
        self._twist = twist
        wx = array(
            [[             0,-self.twist[2], self.twist[1]],
             [ self.twist[2],             0,-self.twist[0]],
             [-self.twist[1], self.twist[0],             0]])
        if self.mass[3,3]==0:
            rx = zeros((3,3))
        else:
            rx = self.mass[0:3,3:6]/self.mass[3,3] # todo: better solution?
        self._nleffects = zeros((6,6))
        self._nleffects[0:3,0:3] = wx
        self._nleffects[3:6,3:6] = wx
        self._nleffects[0:3,3:6] = dot(rx,wx) - dot(wx,rx)
        self._nleffects = dot(self.nleffects, self.mass)

        H_gp = pose
        J_pg = jac
        dJ_pg = djac
        T_pg = twist
        for j in self.childrenjoints:
            H_cn = j._frames[1]._bpose
            H_pr = j._frames[0]._bpose
            H_rn = j.pose()
            H_pc = dot(H_pr, dot(H_rn, Hg.inv(H_cn)))
            child_pose = dot(H_gp, H_pc)
            Ad_cp = Hg.iadjoint(H_pc)
            Ad_cn = Hg.adjoint(H_cn)
            Ad_rp = Hg.adjoint(Hg.inv(H_pr))
            dAd_nr = j.idadjoint()
            dAd_cp = dot(Ad_cn, dot(dAd_nr, Ad_rp))
            T_nr = j.twist()
            J_nr = j.jacobian()
            dJ_nr = j.djacobian()
            child_twist = dot(Ad_cp, T_pg) + dot(Ad_cn, T_nr)
            child_jac = dot(Ad_cp, J_pg)
            child_jac[:,j._dof] += dot(Ad_cn, J_nr)
            child_djac = dot(dAd_cp, J_pg) + dot(Ad_cp, dJ_pg)
            child_djac[:,j._dof] += dot(Ad_cn, dJ_nr)
            j._frames[1].body.update_dynamic(child_pose, child_jac, child_djac, 
                                     child_twist)



def simulate(world, time):

    """
    TODO: add support for collisions

    Example:
    >>> from triplehinge import triplehinge
    >>> w = triplehinge()
    >>> simulate(w, numpy.arange(0,0.01,0.001))
    """

    previous_t = time[0]
    for t in time[1:]:
        dt = t - previous_t
        world.update_dynamic()
        world.update_controllers(dt)
        world.update_constraints(dt)
        world.integrate(dt)


if __name__ == "__main__":
    import doctest
    doctest.testmod()

