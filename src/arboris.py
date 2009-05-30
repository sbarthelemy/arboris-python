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

"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from numpy import ix_, array, zeros, ones, eye, dot
import numpy
import homogeneousmatrix as Hg
import homogeneousmatrix
import twistvector as T
import adjointmatrix
from abc import ABCMeta, abstractmethod, abstractproperty
from math import pi
from controllers import Controller
from misc import NamedObject
from rigidmotion import RigidMotion

class Joint(RigidMotion, NamedObject):
    """A generic class for ideal joints.
    """    
    __metaclass__ = ABCMeta

    def __init__(self, frames=None, name=None):
        NamedObject.__init__(self, name)
        if frames is not None:
            self.attach(frames[0], frames[1])
        else:
            self._frames = (None, None)

    @property
    def frames(self):
        return self._frames

    def attach(self, frame0, frame1):
        self._frames = (frame0, frame1)
        if frame1.body.parentjoint is None:
            frame1.body.parentjoint = self
        else:
            raise ValueError(
                'frame1\'s body already as a parentjoint, which means you\'re probably trying to create a kinematic loop. Try using a constraint instead of a class')
        frame0.body.childrenjoints.append(self)

    @property
    def twist(self):
        return dot(self.jacobian, self.gvel)

    
    @abstractproperty
    def ndof(self):
        """Number of degrees of freedom of the joint
        """
        pass


    @abstractproperty
    def jacobian(self):
        pass


    @abstractproperty
    def djacobian(self):
        pass


    @abstractproperty
    def integrate(self, dt):
        pass

class Constraint(NamedObject):

    def __init__(self, frames, name=None):
        NamedObject.__init__(self, name)

    @abstractmethod
    def gforce(self):
        pass

class BodyConstraint(Constraint):
    __metaclass__ = ABCMeta

    def __init__(self, frames):
        self._frames = frames

    @property
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


class Shape(object):
    """A generic class for geometrical shapes used in collision detectction
    """
    def __init__(self, frame):
        self.frame = frame

class World(NamedObject):
    """

    TODO: provide ability to merge worlds or subtrees
    
    >>> from  triplehinge import triplehinge
    >>> w = triplehinge()
    >>> w.update_geometric()
    >>> w.update_dynamic()
    """

    def __init__(self, name=None):
        NamedObject.__init__(self, name)
        self.ground = Body('ground')
        self._controllers = []
        self._constraints = []
        self._subframes = []
        self._shapes = []
        self._ndof = 0
        self._gvel = array([])
        self._mass =  array([])# updated by self.update_dynamic()
        self._gforce =  array([])
        self._viscosity = array([]) # updated by self.update_dynamic()
        self._controller_viscosity = array([]) # updated by self.update_dynamic()
        self._nleffects = array([]) # updated by self.update_dynamic()


    def iterbodies(self):
        """Iterate over all bodies, with a depth-first strategy"""
        yield self.ground
        for b in self.ground.iter_descendant_bodies():
            yield b


    def itersubframes(self):
        """Iterate over all subframes"""
        for obj in self._subframes:
            yield obj


    def iterframes(self):
        """Iterate over all subframes"""
        for obj in self.iterbodies():
            yield obj
        for obj in self.itersubframes():
            yield obj


    def itershapes(self):
        """Iterate over all shapes"""
        for obj in self._shapes:
            yield obj


    def iterjoints(self):
        """Iterate over all joints, with a depth-first strategy"""
        for j in self.ground.iter_descendant_joints():
            yield j

    def getbodiesdict(self):
        """Returns a dictionary whose values are references to the 
        bodies and keys are the bodies names. Bodies with ``None`` 
        as name will be ignored. Duplicate names will raise a 
        ``DuplicateName`` exception.

        Example:
            >>> from joints import FreeJoint
            >>> w = World()
            >>> #stone = Body('stone')
            >>> #j = FreeJoint()
            >>> #j.attach(w.ground, stone)
            >>> #w.register(j)
            >>> d = w.getbodiesdict()
            >>> d.keys()
            ['ground']
        """
        bodies_dict = {self.ground.name: self.ground}
        for b in self.ground.iter_descendant_bodies():
            if b.name is not None :
                if b.name not in bodies_dict:  
                    bodies_dict[b.name] = b
                else:
                    raise DuplicateNameError()
        return bodies_dict

    def getframesdict(self):
        """Returns a dictionary whose values are references to the
        frames and keys are the frames names. Frames with ``None``
        as name will be ignored. Duplicate names will raise a 
        ``DuplicateName`` exception.

        Example:
            >>> w = World()
            >>> w.register(SubFrame(w.ground, name="ground again"))
            >>> d = w.getframesdict()
            >>> d.keys()
            ['ground again', 'ground']

        """
        frames_dict = self.getbodiesdict()
        for f in self._subframes:
            if f.name is not None :
                if f.name not in frames_dict:  
                    frames_dict[f.name] = f
                else:
                    raise DuplicateNameError()
        return frames_dict

    def getjointsdict(self):
        """Returns a dictionary whose values are references to the 
        joints and keys are the joints names. Frames with ``None`` 
        as name will be ignored. Duplicate names will raise a 
        ``DuplicateName`` exception.

        Example:
            >>> w = World()
            >>> d = w.getjointsdict()
            >>> d.keys()
            []

        """
        joints_dict = {}
        for j in self.ground.iter_descendant_joints():
            if j.name is not None :
                if j.name not in joints_dict:  
                    joints_dict[j.name] = j
                else:
                    raise DuplicateNameError()
        return joints_dict

    def getjointslist(self):
        """Returns a list whose values are references to the 
        joints.

        Example:
            >>> w = World()
            >>> d = w.getjointsdict()
            >>> d.keys()
            []

        """
        joints_list = []
        for j in self.ground.iter_descendant_joints():
            joints_list.append(j)
        return joints_list


    def register(self, obj):
        """
        Register an object into the world.
        
        ``obj`` can be a subframe, a shape or a constraint.
        
        :arguments:
            obj
                the object that will be added. It may be a subframe, a shape,
                a joint, a body or a controller.

        Examples:

        >>> from  triplehinge import triplehinge
        >>> w = triplehinge()
        >>> joints = w.getjointslist()
        >>> joints[0].gvel[0] = 1.
        >>> w._gvel
        array([ 1.,  0.,  0.])
        >>> w._gvel[1] = 2.
        >>> joints[1].gvel
        array([ 2.])

        """
        if isinstance(obj, Body):
            pass
        
        elif isinstance(obj, Joint):
            self.register(obj.frames[0])
            self.register(obj.frames[1])

        elif isinstance(obj, SubFrame):
            if not obj in self._subframes:
                self._subframes.append(obj)

        elif isinstance(obj, Shape):
            if not obj in self._shapes:
                self._shapes.append(obj)
            self.register(obj.frame)

        elif isinstance(obj, Constraint):
            if not obj in self._constraints:
                self._constraints.append(obj)
            if isinstance(obj, BodyConstraint):
                self.register(obj._frames[0])
                self.register(obj._frames[1])

        else:
            raise ValueError()

    
    def initjointspace(self):
        """ Init the joins-space model of the world.
        """
        self._ndof = 0
        for j in self.iterjoints():
            old_ndof = self._ndof
            self._ndof += j.ndof
            j._dof = slice(old_ndof, self._ndof)
            j._dof = slice(old_ndof, self._ndof)

        # Adjust the size of the worldwide model matrices 
        # (we wipe their content)
        self._mass = zeros((self._ndof,self._ndof))
        self._nleffects =  zeros((self._ndof,self._ndof))
        self._viscosity = zeros((self._ndof,self._ndof))
        self._controller_viscosity = zeros((self._ndof,self._ndof))
        self._gforce = zeros(self._ndof)
            
        # Init the worldwide generalized velocity vector:
        # we make self._gvel be a view of a new generalized velocities
        # vector, of the good size.
        self._gvel = zeros(self._ndof)
        # Then, copy the content of each joint generalized velocity 
        # vector to the world one, and then make the joint genralized
        # velocity vector be a (limited) view of the worldwide one.
        # Thus the actual velocity data will be shared in memory between 
        # the world and its joints.
        for j in self.iterjoints():
            self._gvel[j._dof] = j.gvel[:]
            j.gvel = self._gvel[j._dof]

    @property
    def mass(self):
        return self._mass

    @property
    def viscosity(self):
        return self._viscosity

    @property
    def nleffects(self):
        return self._nleffects

    @property
    def ndof(self):
        return self._ndof

    @property
    def gvel(self):
        return self._gvel.copy()



    def add_jointcontroller(self, controller, joints=None):
        """
        Add a joint controller to the world

        TODO: repair this doctest
        Example:

        >> from triplehinge import triplehinge
        >> w = triplehinge()
        >> from controllers import ProportionalDerivativeController
        >> c0 = ProportionalDerivativeController(w.joints[1:3], name = 'my controller')
        >> w.add_jointcontroller(c0, w.joints[1:3])
        >> c1 = ProportionalDerivativeController(w.joints[0:1])
        >> w.add_jointcontroller(c1, w.joints[0:1])
        
        """
        for c in self._controllers:
            if c is controller:
                raise ValueError("the controller is already in the world")

        if joints is None:
            controller._dof = range(self.ndof)
        else:
            controller._dof = []
            for j in joints:
                controller._dof.extend(range(j._dof.start, j._dof.stop))
        
        self._controllers.append(controller)


    def update_geometric(self):
        """
        Compute the forward geometric model. 
        
        This will recursively update each body pose attribute.
        """
        self.ground.geometric(eye(4))


    def update_kinematic(self):
        """
        Compute the forward geometric and kinematic models. 
        
        This will recursively update all each body pose and jacobian
        attributes.
        """
        self.ground.kinematic(eye(4),zeros((6,self._ndof)))


    def update_dynamic(self):
        """
        Compute the forward geometric, kinematic and dynamic models. 
        
        This will recursively update 
        
        - each body pose, jacobian, djacobian, twist and nleffects 
          attributes 
        - the world mass, viscosity and nleffects attributes

        >>> from triplehinge import triplehinge
        >>> w = triplehinge()
        >>> joints  = w.getjointslist()
        >>> joints[0].gpos[0]=0.5
        >>> joints[0].gvel[0]=2.5
        >>> joints[1].gpos[0]=1.0
        >>> joints[1].gvel[0]=-1.0
        >>> joints[2].gpos[0]=2.0/3.0
        >>> joints[2].gvel[0]=-0.5
        >>> w.update_dynamic()
        >>> bodies = w.getbodiesdict()
        >>> bodies['Arm'].pose
        array([[ 0.87758256, -0.47942554,  0.        ,  0.        ],
               [ 0.47942554,  0.87758256,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> bodies['ForeArm'].pose
        array([[ 0.0707372 , -0.99749499,  0.        , -0.23971277],
               [ 0.99749499,  0.0707372 ,  0.        ,  0.43879128],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> bodies['Hand'].pose
        array([[-0.56122931, -0.82766035,  0.        , -0.63871076],
               [ 0.82766035, -0.56122931,  0.        ,  0.46708616],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        >>> bodies['ground'].jacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> bodies['Arm'].jacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 1.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> bodies['ForeArm'].jacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 1.        ,  1.        ,  0.        ],
               [-0.27015115,  0.        ,  0.        ],
               [ 0.42073549,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> bodies['Hand'].jacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 1.        ,  1.        ,  1.        ],
               [-0.26649313, -0.3143549 ,  0.        ],
               [ 0.7450519 ,  0.24734792,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> bodies['ground'].twist
        array([ 0.,  0.,  0.,  0.,  0.,  0.])
        >>> bodies['Arm'].twist
        array([ 0. ,  0. ,  2.5,  0. ,  0. ,  0. ])
        >>> bodies['ForeArm'].twist
        array([ 0.        ,  0.        ,  1.5       , -0.67537788,  1.05183873,  0.        ])
        >>> bodies['Hand'].twist
        array([ 0.        ,  0.        ,  1.        , -0.35187792,  1.61528183,  0.        ])
        >>> w.viscosity
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> bodies['Arm'].mass
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
        >>> bodies['ForeArm'].mass
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
        >>> bodies['Hand'].mass
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
        >>> bodies['ground'].djacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> bodies['Arm'].djacobian
        array([[ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.],
               [ 0.,  0.,  0.]])
        >>> bodies['ForeArm'].djacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [-0.42073549,  0.        ,  0.        ],
               [-0.27015115,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> bodies['Hand'].djacobian
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ],
               [-0.87022993, -0.12367396,  0.        ],
               [-0.08538479, -0.15717745,  0.        ],
               [ 0.        ,  0.        ,  0.        ]])
        >>> bodies['ground'].nleffects
        array([[ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.]])
        >>> bodies['Arm'].nleffects
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
        >>> bodies['ForeArm'].nleffects
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
        >>> bodies['Hand'].nleffects
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
        for b in self.ground.iter_descendant_bodies():
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
        r"""

        Let's consider the following discretized 2nd-order model:

        .. math::
            M(t) \dot\GVel(t+dt) + \left( N(t)+B(t )\right) \GVel(t+dt) &= 
            \GForce(t)

        considering
        
        .. math::
            \dot\GVel(t+dt) = \frac{\GVel(t+dt) - \GVel(t)}{dt}
        
        we get
        
        .. math::
            \left( \frac{M(t)}{dt}+N(t)+B(t) \right) \GVel(t+dt) &= 
            \frac{M(t)}{dt} \GVel(t) + \GForce(t)
        
        Here :math:`\GForce(t)` sums up the generalized forces due to 
        all the active controllers and constraints.

        The generalized force due to a controller has the following form:
        
        .. math::
            \GForce_a(t) &= \GForce_{0a}(t) + Z_a(t) \GVel(t+td)

        where :math:`\GForce_{0a}(t)` is constant during the 
        :math:`[t, t+dt]` period of time.

        It leads us to

        .. math::
            \left( \frac{M(t)}{dt}+N(t)+B(t) -\sum_a Z_a(t) \right) 
            \GVel(t+dt) &= 
            \frac{M(t)}{dt} \GVel(t) + \sum_a \GForce_{0a}(t)

        One can the define impedance (:math:`Z`) and admittance (:math:`Y`)
        matrices:
    
        .. math::
            Z(t) &= \frac{M(t)}{dt}+N(t)+B(t)-\sum_a Z_a(t) \\
            Y(t) &= Z^{-1}(t)


        TODO: check the two last tests results!

        >>> from triplehinge import triplehinge
        >>> w = triplehinge()
        >>> from controllers import ProportionalDerivativeController
        >>> joints = w.getjointslist()
        >>> a0 = ProportionalDerivativeController( joints[1:2], 2.)
        >>> w.add_jointcontroller(a0, joints[1:2])
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
        for a in self._controllers:
            a.update(dt)
            self._controller_viscosity[
                ix_(a._dof, a._dof)] += a.viscosity
            self._gforce[a._dof] += a.gforce
        
        self._impedance = self._mass/dt + self._viscosity + \
            self._nleffects - self._controller_viscosity 
        self._admittance = numpy.linalg.inv(self._impedance)


    def update_constraints(self, dt):
        r"""
        In accordance with the integration scheme, we assume the following
        first order model between generalized velocities and generalized 
        forces:

        .. math::
            \GVel(t+dt) &= Y(t) 
            \left( \frac{M(t)}{dt} \GVel(t) + \GForce(t) \right)

        where:

        - the admittance matrix  :math:`Y` takes into account a 
          first order model of the actuators,

        - the actuators generalized forces :math:`\GForce(t)` 
          are assumed to be constant during the :math:`[t , t+dt ]`
          time interval.
        
        This (constraint-free) model must be completed by constraints 
        forces :math:`\pre[c]f`, which are mapped to generalized forces
        by the constraint jacobian :math:`\pre[c]J_c^T`:

        .. math::
            \GVel(t+dt) 
            &= Y(t) 
            \left( \frac{M(t)}{dt} \GVel(t) + \GForce(t)
                + \sum_{c} \; \pre[c]J_{c}^T(t) \; \pre[c]f(t)
            \right)
  
        one can also define the constraint velocity  as: 
        :math:`\pre[c]v = \pre[c]J_c \; \GVel` so that:
    
        .. math::
            \pre[c]v(t+dt) 
            &= \pre[c]J_c(t) \; \GVel(t+dt)\\
            &= \pre[c]J_c(t) \; Y(t) 
            \left( 
                \frac{M(t)}{dt} \GVel(t) + \; \GForce(t)
            \right)
            + \sum_d \; \pre[c]J_c(t) \; Y(t) \; \pre[d]J_d^T(t) 
            \; \pre[d]f(t)
        
        one can define the (global) constraints velocity :math:`v'`, 
        force :math:`f'`, jacobian matrix :math:`J'` 
        and admittance matrix :math:`Y'`:

        .. math::
            J'(t) &=
            \begin{bmatrix}
                \pre[0]J_0(t)\\
                \vdots\\
                \pre[c]J_c(t)\\
                \vdots
            \end{bmatrix}\\
            v'(t) &= 
            \begin{bmatrix}
                \pre[0]v(t)\\ \vdots \\ \pre[c]v(t) \\ \vdots
            \end{bmatrix}\\
            f'(t) &= 
            \begin{bmatrix}
                \pre[0]f(t)\\ \vdots \\ \pre[c]f(t) \\ \vdots
            \end{bmatrix}\\
            Y'(t) &= 
            J'(t) \; Y(t) \; J'(t)^T

        and get a synthetic expression:

        .. math::
            v'(t+dt) &= J'(t) \; Y(t) 
            \left( \frac{M(t)}{dt}  \GVel(t) + \GForce(t) \right)
            + Y'(t) \; f'


        This method computes the constraint forces in three steps:

        - ask each constraint object for its jacobian,

        - compute :math:`J`, :math:`v`  and :math:`Y`,

        - iterate over each constraint object in order to compute 
          :math:`\pre[c]f`. At each iteration the force is 
          updated by :math:`\Delta\pre[c]f`
        

        Test:

        TODO: the following test fails for an unknown reason, maybe a bug in doctest.

            >> b0 = Body(mass = eye(6))
            >> from joints import FreeJoint
            >> j0 = FreeJoint()
            >> w = World()
            >> j0.attach(w.ground, b0)
            >> j0.register(b0)
            >> b1 = Body(mass = eye(6))
            >> j1 = FreeJoint()
            >> j1.attach(b0, b1)
            >> w.register(b1)
            >> from controllers import WeightController
            >> ctrl =  WeightController(w.ground)
            >> w.add_jointcontroller(ctrl)
            >> from constraints import BallAndSocketConstraint 
            >> c0 = BallAndSocketConstraint()
            >> c0._frames = (w.ground, b0)
            >> w.register(c0)
            >> w.update_dynamic()
            >> dt = 0.001
            >> w.update_controllers(dt)
            >> w.update_constraints(dt)
            >> c0._force
            array([ 0.  ,  9.81,  0.  ])
            >> w.integrate(dt)
            >> w.update_dynamic()
            >> b0.pose
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
            c.update()
            if c.is_active():
                c._dol = slice(ndol, ndol+c.ndol())
                ndol = ndol + c.ndol()
                constraints.append(c)

        jac = zeros((ndol, self._ndof))
        admittance = zeros((ndol, ndol))
        for c in constraints:
            jac[c._dol,:] = c.jacobian()
        vel = dot(jac, dot(self._admittance, 
                           dot(self._mass, self._gvel/dt) + self._gforce))
        admittance = dot(jac, dot(self._admittance, jac.T))

        k=0
        while k < 20: 
        #TODO: change the break condition, it should be computed from the error
            k+=1 
            for c in constraints:
                dforce = c.solve(vel[c._dol], admittance[c._dol,c._dol], dt)
                vel += dot(admittance[:,c._dol], dforce)
        for c in constraints:
            self._gforce += c.gforce()


    def integrate(self, dt):
        r"""
        
        From the :method:`update_controllers` and 
        :method:`update_constraints` methods we get the new 
        generalized velocity.

        .. math::
            \GVel(t+dt) &= Y(t) \left(
            \frac{M(t)}{dt} \GVel(t) 
            + \sum_{c=\text{controllers}} \GForce_{0c}(t)
            + \sum_{c=\text{constraints}} \GForce_{c}(t)
            \right)

        In order to get the new generalized position, each joint is integrated
        separately.

        TODO: add support for kinematic controllers
        TODO: repair this doctest
        TODO: check the last test result!

        >> from triplehinge import triplehinge
        >> w = triplehinge()
        >> joints = w.getjointsdict()
        >> joints['Shoulder'].gpos[:] = -1.
        >> from controllers import ProportionalDerivativeController
        >> c0 = ProportionalDerivativeController(w.joints[1:2], 1.)
        >> w.add_jointcontroller(c0, w.joints[1:2])
        >> w.update_dynamic()
        >> dt = 0.001
        >> w.update_controllers(dt)
        >> w.integrate(dt)
        >> w._gvel
        array([-0.00709132,  0.03355273, -0.09131555])
        """
        self._gvel[:] = dot(self._admittance, 
                            dot(self._mass, self._gvel/dt) + self._gforce)
        for j in self.iterjoints():
            j.integrate(dt)


class Frame(object):
    __metaclass__ = ABCMeta

    @abstractproperty
    def pose(self):
        pass

    @abstractproperty
    def jacobian(self):
        pass

    @abstractproperty
    def twist(self):
        pass

    @abstractproperty
    def body(self):
        pass

    @abstractproperty
    def bpose(self):
        pass


class SubFrame(NamedObject, Frame):

    def __init__(self, body, bpose=None, name=None):
        """Create a frame rigidly fixed to a body. 
        
        >>> b = Body()
        >>> f = SubFrame(b,Hg.rotz(3.14/3.),'Brand New Frame')
        
        The ``body`` argument must be a member of the ``Body`` class:
        >>> f = SubFrame(None, Hg.rotz(3.14/3.))
        Traceback (most recent call last):
            ...
        ValueError: The ``body`` argument must be an instance of the ``Boby`` class

        The ``bpose`` argument must be an homogeneous matrix:
        >>> b = Body()
        >>> f = SubFrame(b, ones((4,4)))
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
            self._body = body

    @property
    def pose(self):
        return dot(self._body.pose, self._bpose)
    
    @property
    def twist(self):
        return dot(Hg.iadjoint(self._bpose), self._body._twist)

    @property
    def jacobian(self):
        return dot(Hg.iadjoint(self._bpose), self._body._jacobian)

    @property
    def body(self):
        return self._body

    @property
    def bpose(self):
        return self._bpose.copy()


class Body(NamedObject, Frame):

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
        self.parentjoint = None
        self.childrenjoints = []
        self.mass = mass
        self.viscosity = viscosity
        self._pose = None # updated by update_{geometric,kinematic,dynamic}
        self._jacobian = None # updated by update_{geometric,kinematic,dynamic}
        self._djacobian = None # updated by update_dynamic
        self._twist = None # updated by update_dynamic
        self._nleffects = None # updated by update_dynamic

    def iter_descendant_bodies(self):
        """Iterate over all descendant bodies, with a depth-first strategy"""
        from itertools import imap
        for b in imap(lambda j: j._frames[1].body, self.childrenjoints):
            yield b
            for bb in b.iter_descendant_bodies():
                yield bb

    def iter_ancestor_bodies(self):
        if self.parentjoint is not None:
            parentbody = self.parentjoint._frames[0].body
            yield parentbody
            for a in parentbody.iter_ancestor_bodies():
                yield a
    
    def iter_descendant_joints(self):
        """Iterate over all descendant joints, with a depth-first strategy"""
        for j in self.childrenjoints:
            yield j
            for jj in j._frames[1].body.iter_descendant_joints():
                yield jj

    def iter_ancestor_joints(self):
        if self.parentjoint is not None:
            yield self.parentjoint
            for a in self.parentjoint._frames[0].body.iter_ancestor_joints():
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

    @property
    def bpose(self):
        return eye(4)

    @property
    def body(self):
        return self

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
            H_cn = j._frames[1].bpose
            H_pr = j._frames[0].bpose
            H_rn = j.pose
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
                + Ad_cp * J_pg * ddQ 
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
            rx = self.mass[0:3,3:6]/self.mass[3,3] #TODO: better solution?
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
            H_cn = j._frames[1].bpose
            H_pr = j._frames[0].bpose
            H_rn = j.pose
            H_pc = dot(H_pr, dot(H_rn, Hg.inv(H_cn)))
            child_pose = dot(H_gp, H_pc)
            Ad_cp = Hg.iadjoint(H_pc)
            Ad_cn = Hg.adjoint(H_cn)
            Ad_rp = Hg.adjoint(Hg.inv(H_pr))
            dAd_nr = j.idadjoint
            dAd_cp = dot(Ad_cn, dot(dAd_nr, Ad_rp))
            T_nr = j.twist
            J_nr = j.jacobian
            dJ_nr = j.djacobian
            child_twist = dot(Ad_cp, T_pg) + dot(Ad_cn, T_nr)
            child_jac = dot(Ad_cp, J_pg)
            child_jac[:,j._dof] += dot(Ad_cn, J_nr)
            child_djac = dot(dAd_cp, J_pg) + dot(Ad_cp, dJ_pg)
            child_djac[:,j._dof] += dot(Ad_cn, dJ_nr)
            j._frames[1].body.update_dynamic(child_pose, child_jac, child_djac, 
                                     child_twist)



def simulate(world, time):

    """
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

