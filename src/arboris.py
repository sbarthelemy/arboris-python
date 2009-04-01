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

- remove left/right-frames from joints
- make use of @property for read-only properties
- add support for FreeJoint
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

__version__ = "$Revision: 1 $"
# $Source$

import numpy as np
import homogeneousmatrix as Hg
import homogeneousmatrix
import twistvector as T
import adjointmatrix

Pi = 3.14 #TODO: improve precision

class World(object):

    """

    TODO: provide ability to merge worlds or subtrees
    
    >>> import worldfactory
    >>> w = worldfactory.triplehinge()
    >>> w.geometric()
    >>> w.dynamic()
    """

    def __init__(self,name=None):
        self.name = unicode(name)
        self.bodies = [Body(u"ground")] 
        self.joints = [] 
        self._ndof = 0
        self._mass = None # updated by self.dynamic
        self._viscosity = None # updated by self.dynamic
        self._nleffect = None # updated by self.dynamic

    @property
    def mass(self):
        return self._mass

    @property
    def viscosity(self):
        return self._viscosity

    @property
    def nleffect(self):
        return self._nleffect

    def reset(self):
        """
        Set all state-dependent data to None
        """
        self._mass = None
        self._viscosity = None
        self._nleffect = None
        for b in self.bodies:
            b.reset()


    def ndof(self):
        return self._ndof


    def addjoint(self, joint):
        """Add a joint and its new-attached body to the world.

        :arguments:
            joint
                the joint that will be added
        """
        # check the left frame is already in world
        ref_frame_is_in_world = False
        for f in self.bodies:
            if joint.ref_frame.body is f:
               ref_frame_is_in_world = True
        if not(ref_frame_is_in_world):
            raise ValueError
        
        newbody = joint.new_frame.body
        # check the new frame is not already in world
        new_frame_is_in_world = False
        for f in self.bodies:
            if newbody is f:
               raise ValueError

        if (newbody.parentjoint != None):
            raise ValueError
        if newbody.childrenjoints != []:
            raise ValueError
        
        # add the joint and the moving frame to the world
        self.joints.append(joint)
        joint.ref_frame.body.childrenjoints.append(joint)
        self.bodies.append(newbody)
        newbody.parentjoint = joint

        # extend the world generalized velocities
        old_ndof = self._ndof
        self._ndof = self._ndof + joint.ndof()
        joint.dofindex = slice(old_ndof, self._ndof)
        joint.gvel = np.array(joint.gvel).reshape(-1) # ensure gvel is a 1-d array
        

    def geometric(self):
        """
        Compute the forward geometric model. 
        
        This will recursively update each body pose attribute.
        """
        self.bodies[0].geometric(np.eye(4))


    def kinematic(self):
        """
        Compute the forward geometric and kinematic models. 
        
        This will recursively update all each body pose and jacobian
        attributes.
        """
        self.bodies[0].kinematic(np.eye(4),np.zeros((6,self._ndof)))


    def dynamic(self):
        """
        Compute the forward geometric, kinematic and dynamic models. 
        
        This will recursively update 
        
        - each body pose, jacobian, djacobian, twist and nleffect 
          attributes 
        - the world mass, viscosity and nleffect attributes

        >>> import worldfactory
        >>> w = worldfactory.triplehinge()
        >>> w.joints[0].gpos[0]=0.5
        >>> w.joints[0].gvel[0]=2.5
        >>> w.joints[1].gpos[0]=1.0
        >>> w.joints[1].gvel[0]=-1.0
        >>> w.joints[2].gpos[0]=2.0/3
        >>> w.joints[2].gvel[0]=-0.5
        >>> w.dynamic()
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
        >>> w.bodies[0].nleffect
        array([[ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.],
               [ 0.,  0.,  0.,  0.,  0.,  0.]])
        >>> w.bodies[1].nleffect
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
        >>> w.bodies[2].nleffect
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
        >>> w.bodies[3].nleffect
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
        >>> w.nleffect
        array([[ 0.11838112, -0.15894538, -0.01490104],
               [ 0.27979997,  0.00247348, -0.00494696],
               [ 0.03230564,  0.00742044,  0.        ]])

        """        
        self.bodies[0].dynamic(
            np.eye(4),
            np.zeros((6,self._ndof)),
            np.zeros((6,self._ndof)),
            np.zeros(6))
        
        self._mass = np.zeros((self._ndof,self._ndof))
        self._viscosity = np.zeros((self._ndof,self._ndof))
        self._nleffect = np.zeros((self._ndof,self._ndof))
        for b in self.bodies:
            self._mass += np.dot(
                np.dot(b.jacobian.transpose(), b.mass),
                b.jacobian)
            self._viscosity += np.dot(
                np.dot(b.jacobian.transpose(), b.viscosity),
                b.jacobian)
            self._nleffect += np.dot(
                b.jacobian.transpose(),
                np.dot(
                    b.mass,
                    b.djacobian) \
                + np.dot(
                    b.viscosity+b.nleffect,
                    b.jacobian))


class Frame(object):

    def __init__(self, body, pose=None, name=None):
        """Create a frame rigidly fixed to a body. 
        
        >>> b = Body()
        >>> f = Frame(b,Hg.rotz(Pi/3),'Brand New Frame')
        
        The ``body`` argument must be a member of the ``Body`` class:
        >>> f = Frame(None, Hg.rotz(Pi/3))
        Traceback (most recent call last):
            ...
        ValueError: The ``body`` argument must be an instance of the ``Boby`` class

        The ``pose`` argument must be an homogeneous matrix:
        >>> b = Body()
        >>> f = Frame(b, np.ones((4,4)))
        Traceback (most recent call last):
            ...
        ValueError: [[ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]
         [ 1.  1.  1.  1.]] is not an homogeneous matrix

        """
        if pose == None:
            pose = np.eye(4)
        self.name = unicode(name)
        Hg.checkishomogeneousmatrix(pose)
        self.pose = pose
        if not(isinstance(body,Body)):
            raise ValueError("The ``body`` argument must be an instance of the ``Boby`` class")
        else:
            self.body = body


class Body(object):

    def __init__(self, name=None, mass=None, viscosity=None):
        if mass == None:
            mass = np.zeros((6,6))
        else:
            pass #TODO: check the mass matrix
        if viscosity == None:
            viscosity = np.zeros((6,6))
        else:
            pass #TODO: check the viscosity matrix

        self.name = unicode(name)
        self.frames = [Frame(self, np.eye(4), unicode(name))]
        self.parentjoint = None
        self.childrenjoints = []
        self.mass = mass
        self.viscosity = viscosity
        self._pose = None # updated by self.geometric()
        self._jacobian = None # updated by self.kinematic()/self.dynamic()
        self._djacobian = None # updated by self.dynamic()
        self._twist = None # updated by self.dynamic() 
        self._nleffect = None # updated by self.dynamic() 

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
    def nleffect(self):
        return self._nleffect


    def reset(self):
        self._pose = None
        self._jacobian = None
        self._djacobian = None
        self._twist = None
        self._nleffect = None

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
            H_cn = j.new_frame.pose
            H_pr = j.ref_frame.pose
            H_rn = j.pose()
            H_pc = np.dot(H_pr, np.dot(H_rn, Hg.inv(H_cn)))
            child_pose = np.dot(H_gp, H_pc)
            j.new_frame.body.geometric(child_pose)
        
    def kinematic(self,pose,jac):
        raise NotImplemented #TODO: remove the method or implement it
    
    def dynamic(self,pose,jac,djac,twist):
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
        wx = np.array(
            [[             0,-self.twist[2], self.twist[1]],
             [ self.twist[2],             0,-self.twist[0]],
             [-self.twist[1], self.twist[0],             0]])
        if self.mass[3,3]==0:
            rx = np.zeros((3,3))
        else:
            rx = self.mass[0:3,3:6]/self.mass[3,3] # todo: better solution?
        self._nleffect = np.zeros((6,6))
        self._nleffect[0:3,0:3] = wx
        self._nleffect[3:6,3:6] = wx
        self._nleffect[0:3,3:6] = np.dot(rx,wx)-np.dot(wx,rx)
        self._nleffect = np.dot(self.nleffect,self.mass)

        H_gp = pose
        J_pg = jac
        dJ_pg = djac
        T_pg = twist
        for j in self.childrenjoints:
            H_cn = j.new_frame.pose
            H_pr = j.ref_frame.pose
            H_rn = j.pose()
            #H_nr = j.ipose()
            H_pc = np.dot(H_pr, np.dot(H_rn, Hg.inv(H_cn)))
            child_pose = np.dot(H_gp, H_pc)
            Ad_cp = Hg.iadjoint(H_pc)
            Ad_cn = Hg.adjoint(H_cn)
            Ad_rp = Hg.adjoint(Hg.inv(H_pr))
            dAd_nr = j.idadjoint()
            dAd_cp = np.dot(Ad_cn, np.dot(dAd_nr, Ad_rp))
            T_nr = j.twist()
            J_nr = j.jacobian()
            dJ_nr = j.djacobian()
            child_twist = np.dot(Ad_cp, T_pg) + np.dot(Ad_cn, T_nr)
            child_jac = np.dot(Ad_cp, J_pg)
            child_jac[:,j.dofindex] += np.dot(Ad_cn, J_nr)
            child_djac = np.dot(dAd_cp, J_pg) + np.dot(Ad_cp, dJ_pg)
            child_djac[:,j.dofindex] += np.dot(Ad_cn, dJ_nr)
            j.new_frame.body.dynamic(child_pose, child_jac, child_djac, 
                                      child_twist)


class RigidMotion(object):

    """Model a rigid motion, including relative pose and velocity

    n: new frame
    r: reference frame
    self.pose(): H_rn
    self.ipose(): H_nr
    self.twist(): T_nr
    self.itwist(): T_rn
    self.adjoint(): Ad_rn == adjoint(H_rn)
    self.iadjoint(): Ad_nr == adjoint(H_nr)
    self.adjacency(): adjacency(T_nr)
    self.iadjacency(): adjacency(T_rn)
    self.dadjoint(): dAd_rn
    self.idadjoint():dAd_nr
    """
    
    def pose(self):
        """Return the pose as an homogeneous matrix. 
        
        This method must be overloaded.        
        """
        raise NotImplementedError #TODO: make use of python2.6's ABC

    def ipose(self):
        """Inverse of pose()
        """
        return Hg.inv(self.pose())
        
    def twist(self):
        """Return the velocity as a twist vector. 
        
        This method must be overloaded.        
        """
        raise NotImplementedError #TODO: make use of python2.6's ABC

    def itwist(self):
        return -np.dot(np.asarray(self.iadjoint()),self.twist())

    def adjoint(self):
        return Hg.adjoint(self.pose())

    def iadjoint(self):
        return Hg.adjoint(self.ipose())

    def adjacency(self):
        return T.adjacency(self.twist())

    def iadjacency(self):
        return T.adjacency(self.itwist())


    def dadjoint(self):
        return np.dot(np.asarray(self.adjoint()),self.adjacency())
    
    def idadjoint(self):
        return np.dot(np.asarray(self.iadjoint()),self.iadjacency())

class Joint(RigidMotion):

    """any joint
    H
    T_
    """    
    
    def __init__(self, ref_frame, new_frame, name=None):
        if not(isinstance(ref_frame,Frame)):
            raise ValueError()
        if not(isinstance(new_frame,Frame)):
            raise ValueError()
        self.name = unicode(name)
        self.ref_frame = ref_frame
        self.new_frame = new_frame
        self.dofindex = None # will be set when the joint is added to the world

    def twist(self):
        return np.dot(self.jacobian(),self.gvel)

    def ndof(self):
        """Number of degrees of freedom of the joint

        This method must be overloaded.
        """
        raise NotImplementedError #TODO: make use of python2.6's ABC

    def jacobian(self):
        """

        This method must be overloaded.
        """
        raise NotImplementedError #TODO: make use of python2.6's ABC

    def djacobian(self):
        """

        This method must be overloaded.
        """
        raise NotImplementedError #TODO: make use of python2.6's ABC


class FreeJoint(Joint):

    """Free joint (6-dof)
    """


class PivotJoint(Joint):

    """Pivot (2-dof)
    """


class BallJoint(Joint):

    """Ball and socket (3-dof)
    """

class HingeJoint(Joint):

    """Hinge (1-dof) with axis in the z-direction
    """
    def __init__(self, ref_frame, new_frame, name=None, 
                 gpos=0., gvel=0.):
        Joint.__init__(self, ref_frame, new_frame, name)
        self.gpos = np.array(gpos).reshape((1))
        self.gvel = np.array(gvel).reshape((1))
    
    def ndof(self):
        return 1

    def pose(self):
        return Hg.rotz(self.gpos[0])

    def ipose(self):
        return Hg.rotz(-self.gpos[0])

    def jacobian(self):
        return np.array([[0.], [0.], [1.], [0.], [0.], [0.]])

    def djacobian(self):
        return np.zeros((6,1))


class Simulation(object):
   
    """A simulation
    Just a placeholder, not working yet !
    """
    def __init__(self,world):
        self.world = world
        self.time = (0,1,2,3,4)

    def run(self):

        for t in time:

            # compute the world model
            self.world.geometric()
            self.world.dynamic()

            # compute torques and wrenches from controllers
            #controlers...

            #integrate without contacts wrenches
            world2 = self.world.copy()
            world2 = world2.integrate()

            # compute explicit contraints wrenches
            world2.geometric()
            wr = contact_wrenches(world2)

            # integrate back
            self.world.integrate(wr)

            # emit signal, for visu or snapshot saving...

if __name__ == "__main__":
    import doctest
    doctest.testmod()

