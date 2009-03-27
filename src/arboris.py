# coding=utf-8
"""
...

A world consists of bodies (instances of the :class:`Body` class) interlinked
by joints (instances of :class:`Joint` subclasses). It forms a tree whose 
bodies are the nodes and joints are the edges.

One or more frames (instances of the :class:`Frame` class) can be associated to
bodies and serve as anchor points to the joints

TODO: 

- add support for controllers and integration
- split world description and its state+matrices ? 
- add support for visu
- add support for explicit joints
- add support for contacts and joint limits
- add support for coupled joints
- add support for non-holonomic joints

"""

__version__ = "$Revision: 1 $"
# $Source$

import numpy as np
import homogeneousmatrix as Hg
import twistvector as T

Pi = 3.14

class World(object):

    """

    TODO: provide ability to merge worlds or subtrees
    
    >>> import worldfactory
    >>> w = worldfactory.triplehinge()
    >>> w.geometric()
    >>> w.kinematic()
    >>> w.dynamic()
    """

    def __init__(self,name=None):
        self.name = unicode(name)
        self.bodies = [Body(u"ground")] 
        self.joints = [] 
        self._ndof = 0
        self.gvel = np.zeros(0) # concatenates all joints gvel
        self.mass = None # updated by self.dynamic
        self.viscosity = None # updated by self.dynamic
        self.nleffect = None # updated by self.dynamic


    def reset(self):
        """
        Set all state-dependent data to None
        """
        self.mass = None
        self.viscosity = None
        self.nleffect = None
        for b in self.bodies:
            b.reset()


    def ndof(self):
        return self._ndof


    def addjoint(self, joint):
        """Add a joint and its right-attached body to the world.
        """
        # check the left frame is already in world
        left_frame_is_in_world = False
        for f in self.bodies:
            if joint.leftframe.body is f:
               left_frame_is_in_world = True
        if not(left_frame_is_in_world):
            raise ValueError
        
        newbody = joint.rightframe.body
        # check the new frame is not already in world
        right_frame_is_in_world = False
        for f in self.bodies:
            if newbody is f:
               raise ValueError

        if (newbody.parentjoint != None):
            raise ValueError
        if newbody.childrenjoints != []:
            raise ValueError
        
        # add the joint and the moving frame to the world
        self.joints.append(joint)
        joint.leftframe.body.childrenjoints.append(joint)
        self.bodies.append(newbody)
        newbody.parentjoint = joint

        # extend the world generalized velocities
        old_gvel = self.gvel
        old_ndof = self._ndof
        self._ndof = self._ndof + joint.ndof()
        self.gvel = np.zeros(self._ndof)
        self.gvel[0:old_ndof] = old_gvel        
        
        joint.dofindex = slice(old_ndof, self._ndof)
        self.gvel[joint.dofindex] = joint.gvel
        # make joint.gvel a view of self.gvel
        joint.gvel = self.gvel[joint.dofindex] 
        

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
        
        """        
        self.bodies[0].dynamic(
            np.eye(4),
            np.zeros((6,self._ndof)),
            np.zeros((6,self._ndof)),
            np.zeros(6))
        
        self.mass = np.zeros((self._ndof,self._ndof))
        self.viscosity = np.zeros((self._ndof,self._ndof))
        self.nleffect = np.zeros((self._ndof,self._ndof))
        for b in self.bodies:
            self.mass += np.dot(
                np.dot(b.jacobian.transpose(), b.mass),
                b.jacobian)
            self.viscosity += np.dot(
                np.dot(b.jacobian.transpose(), b.viscosity),
                b.jacobian)
            self.nleffect += np.dot(
                b.jacobian.transpose(),
                np.dot(
                    b.mass,
                    b.djacobian) \
                + np.dot(
                    b.viscosity+b.nleffect,
                    b.jacobian))


class Frame(object):

    def __init__(self, body, pose=np.eye(4), name=None):
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
        self.name = unicode(name)
        Hg.checkishomogeneousmatrix(pose)
        self.pose = pose
        if not(isinstance(body,Body)):
            raise ValueError("The ``body`` argument must be an instance of the ``Boby`` class")
        else:
            self.body = body


class Body(object):

    def __init__(self, name=None, mass=np.zeros((6,6)), 
                 viscosity=np.zeros((6,6))):
        self.name = unicode(name)
        self.frames = [Frame(self, np.eye(4), unicode(name))]
        self.parentjoint = None
        self.childrenjoints = []
        self.mass = mass
        self.viscosity = viscosity
        self.pose = None # updated by self.geometric()
        self.jacobian = None # updated by self.kinematic()/self.dynamic()
        self.djacobian = None # updated by self.dynamic()
        self.twist = None # updated by self.dynamic() 
        self.nleffect = None # updated by self.dynamic() 

    def reset(self):
        self.pose = None
        self.jacobian = None
        self.djacobian = None
        self.twist = None
        self.nleffect = None

    def newframe(self,pose,name=None):
        frame = Frame(self,pose,name)
        self.frames.append(frame)
        return frame
        
    def geometric(self,pose):
        self.pose = pose
        for j in self.childrenjoints:
            H = np.dot(j.leftframe.pose,
                np.dot(j.pose(), Hg.inv(j.rightframe.pose)))
            j.rightframe.body.geometric( np.dot(pose, H))
        
    def kinematic(self,pose,jac):
        self.pose = pose
        self.jacobian = jac
        for j in self.childrenjoints:
            H = np.dot(j.leftframe.pose,
                np.dot(j.pose(), Hg.inv(j.rightframe.pose)))
            child_pose = np.dot(pose, H)
            child_jac = np.dot(Hg.adjoint(H), jac)
            child_jac[:,j.dofindex] += np.dot(
                Hg.adjoint(j.leftframe.pose),
                j.jacobian())
            j.rightframe.body.kinematic(child_pose,child_jac)

    def dynamic(self,pose,jac,djac,twist):
        self.pose = pose
        self.jacobian = jac
        self.djacobian = djac
        self.twist = twist

        wx = np.array(
            [[             0,-self.twist[2], self.twist[1]],
             [ self.twist[2],             0,-self.twist[0]],
             [-self.twist[1], self.twist[0],             0]])
        if self.mass[3,3]==0:
            rx = np.zeros((3,3))
        else:
            rx = self.mass[3:6,0:3]/self.mass[3,3] # todo: better solution?
        self.nleffect = np.zeros((6,6))
        self.nleffect[0:3,0:3] = wx
        self.nleffect[3:6,3:6] = wx
        self.nleffect[0:3,3:6] = np.dot(rx,wx)-np.dot(wx,rx)
        self.nleffect = np.dot(self.nleffect,self.mass)

        for j in self.childrenjoints:
            H = np.dot(
                j.leftframe.pose,
                np.dot(j.pose(), Hg.inv(j.rightframe.pose)))
            child_pose = np.dot(pose, H)
            Ad = Hg.adjoint(H)
            dAd = np.dot(
                Hg.adjoint(j.leftframe.pose),
                np.dot(
                    j.dadjoint(),
                    Hg.adjoint(Hg.inv(j.rightframe.pose))))
            child_twist = twist + np.dot(
                Hg.adjoint(j.leftframe.pose),
                j.twist())
            child_jac = np.dot(Ad, jac)
            child_jac[:,j.dofindex] += np.dot(
                Hg.adjoint(j.leftframe.pose),
                j.jacobian())
            child_djac = np.dot(Ad, djac) + np.dot(dAd,jac)
            child_djac[:,j.dofindex] += np.dot(
                Hg.adjoint(j.leftframe.pose),
                j.djacobian())
            j.rightframe.body.dynamic(child_pose, child_jac, child_djac, 
                                      child_twist)


class RigidMotion(object):

    """Model a rigid motion, including relative pose and velocity
    """
    
    def pose(self):
        return self._pose

    def twist(self):
        return self._twist

    def adjoint(self):
        return Hg.adjoint(self.pose())

    def adjacency(self):
        return T.adjacency(self.twist())

    def dadjoint(self):
        return np.dot(np.asarray(self.adjoint()),self.adjacency())
    

class Joint(RigidMotion):

    """any joint
    """    
    
    def __init__(self, leftframe, rightframe, name=None):
        if not(isinstance(leftframe,Frame)):
            raise ValueError()
        if not(isinstance(rightframe,Frame)):
            raise ValueError()
        self.name = unicode(name)
        self.leftframe = leftframe
        self.rightframe = rightframe
        self.dofindex = None # will be set when the joint is added to the world

    def twist(self):
        return np.dot(self.jacobian(),self.gvel)


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
    def __init__(self, leftframe, rightframe, name=u"", gpos=0, gvel=0):
        Joint.__init__(self, leftframe, rightframe, name)
        self.gpos = gpos
        self.gvel = gvel
    
    def ndof(self):
        return 1

    def pose(self):
        return Hg.rotz(self.gpos)

    def jacobian(self):
        return np.array([[0], [0], [1], [0], [0], [0]])

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

