"""
MovingFrame and rigid body motions
"""

import numpy as np
import homogeneousmatrix as htr
import adjointmatrix

class HomogeneousMatrix(np.ndarray):

    def __new__(subtype, data=np.eye(4)):
        subarr = np.array(data)
        if htr.ishomogeneousmatrix(subarr):
            return subarr.view(subtype)
        else:
            raise ValueError
    
    def inv(self):
        """
        Invert the homogenenous matrix
        """
        self[0:4,0:4] = htr.inv(self)
    
    def adjoint(self):
        """
        Return the adjoint of the homogeneous matrix
        """
        return AdjointMatrix(htr.adjoint(self))


class AdjointMatrix(np.ndarray):
    
    def __new__(subtype, data=np.eye(4)):
        subarr = np.array(data)
        if adjointmatrix.isadjointmatrix(subarr):
            return subarr.view(subtype)
        else:
            raise ValueError
    
    def inv(self):
        """
        Invert the adjoint matrix
        """
        self[0:6,0:6] = adjointmatrix.inv(self)


class Twist(np.ndarray):

    def __new__(subtype, data=np.zeros(6)):
        subarr = np.array(data)
        subarr.resize(6)
        return subarr.view(subtype)
    
    def transport(self,H):
        self[:] = reduce(self,H)
    
    def reduce(self,H):
        """
        Return the numerical expression of the twist, expressed at H
        """
        H=HomogeneousMatrix(H)
        return np.dot(H.adjoint(),self)
 

class Wrench(np.ndarray):

    def __new__(subtype, data=np.zeros(6)):
        subarr = np.array(data)
        subarr.resize(6)
        return subarr.view(subtype)
    
    def transport(self,H):
        self[:] = reduce(self,H)
    
    def reduce(self,H):
        """
        Return the numerical expression of the wrench, expressed at H
        """
        H=HomogeneousMatrix(H)
        return np.dot(self,H.adjoint())


class World(object):


    def __init__(self):
        self.bodies = [Body(u"ground")] 
        self.joints = [] 
        self._ndof = 0
        self.gvel = np.zeros(0)

    def ndof(self):
        return self._ndof

    def addjoint(self, joint):
        
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
        self.bodies[0].geometric(np.eye(4))

    def kinematic(self):
        self.bodies[0].kinematic(np.zeros((6,self._ndof)))
        


class Frame:

    def __init__(self, body, pose=np.eye(4), name=None):
        self.name = unicode(name)
        if not htr.ishomogeneousmatrix(pose):
            raise ValueError()
        self.pose = pose
        if not(isinstance(body,Body)):
            raise ValueError
        else:
            self.body = body


class Body:

    def __init__(self, name=None):
        self.name = unicode(name)
        self.frames = [Frame(self, np.eye(4), u"body")]
        self.parentjoint = None
        self.childrenjoints = []
        self.pose = None # updated by self.geometric()

    def addframe(self,pose,name=None):
        self.frames.append(Frame(self,pose,name))

    def geometric(self,pose):
        self.pose = pose
        for j in self.childrenjoints:
            H = np.dot(j.leftframe.pose,
                np.dot(j.pose(), htr.inv(j.rightframe.pose)))
            j.rightframe.body.geometric( np.dot(pose, H))
        
    def kinematic(self,jac):
        self.jacobian = jac
        for j in self.childrenjoints:
            H = np.dot(j.leftframe.pose,
                np.dot(j.pose(), htr.inv(j.rightframe.pose)))
            jac = np.dot(htr.adjoint(H), jac)
            jac[:,j.dofindex] += np.dot(htr.adjoint(j.leftframe.pose),
                j.jacobian())
            j.rightframe.body.kinematic(jac)
                    



class Transform(object):
    """
    Matrix from SE(3)
    """
    def __init__(self):
        self.pose = np.eye(4)
        self.vel = np.zeros((6,1))
#        self.acc = np.zeros((6,1))
#        self.adjoint = np.eye(6)
#        self.adjacency = np.eye(6)

    def set_pose(self,H):
        self.pose       = H
#        self.adjoint   = H 

    def set_state(self):
        pass
    
    def gethomogeneousmatrix(self):
        return self.pose
    
    def set_vel(self,V):
        self.vel = V
    
    def get_twist(self):
        return self.vel
    
    def integrate(self,dQt):
        pass


class Joint(object):
    """
    any joint
    """    
    def __init__(self, leftframe, rightframe, name = u""):
        if not(isinstance(leftframe,Frame)):
            raise ValueError()
        if not(isinstance(rightframe,Frame)):
            raise ValueError()
        self.name = unicode(name)
        self.leftframe = leftframe
        self.rightframe = rightframe
        self.dofindex = None # will be set when the joint is added to the world

class FreeJoint(Joint):
    """
    Free joint (6-dof)
    """


class PivotJoint(Joint):
    """
    Pivot (2-dof)
    """


class BallJoint(Joint):
    """
    Ball and socket (3-dof)
    """

class HingeJoint(Joint):
    """
    Hinge (1-dof)
    """
    def __init__(self, leftframe, rightframe, name=u"", gpos=0, gvel=0):
        Joint.__init__(self, leftframe, rightframe, name)
        self.gpos = gpos
        self.gvel = gvel
    
    def ndof(self):
        return 1

    def pose(self):
        return htr.rotz(self.gpos)

    def jacobian(self):
        return np.array([[0], [0], [1], [0], [0], [0]])

