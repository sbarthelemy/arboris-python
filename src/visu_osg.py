# coding=utf-8
"""
Visualization of a simulation
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from OpenSceneGraph import osg, osgDB, osgGA, osgViewer, osgText # on linux
#import osg, osgDB, osgGA, osgViewer, osgText                    # on windows
import numpy as np
import visu


def pose_2_mat(pose):
    m = osg.Matrixd()
    m.set(pose[0,0], pose[1,0], pose[2,0], pose[3,0],
          pose[0,1], pose[1,1], pose[2,1], pose[3,1],
          pose[0,2], pose[1,2], pose[2,2], pose[3,2],
          pose[0,3], pose[1,3], pose[2,3], pose[3,3],
          ) # here we do the f***ing transposition: beware of error
    return m


#we create the frame node and we will re-use it for every frame created
def create_generic_frame(scale=1.):
    # TODO: see for a less complicated frame conception (with triangle e.g.)
    # WARN: we create the leaves and go towards the trunk of the osg tree
    # create the x cylinder
    cyl_x = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,scale/2.), 0.05*scale, scale))
    cyl_y = osg.ShapeDrawable(cyl_x)
    cyl_z = osg.ShapeDrawable(cyl_x)
    cyl_x.setColor(osg.Vec4(1.,0.,0.,1.))
    cyl_y.setColor(osg.Vec4(0.,1.,0.,1.))
    cyl_z.setColor(osg.Vec4(0.,0.,1.,1.))
    # create the geo node for each cylinder
    geo_x = osg.Geode()
    geo_x.addDrawable( cyl_x )
    geo_y = osg.Geode()
    geo_y.addDrawable( cyl_y )
    geo_z = osg.Geode()
    geo_z.addDrawable( cyl_z )
    # create the transform node for geo_x and geo_y
    trans_x = osg.PositionAttitudeTransform()
    trans_x.setAttitude(osg.Quat(np.pi/2., osg.Vec3(0.,1.,0.)))
    trans_x.addChild(geo_x)
    trans_y = osg.PositionAttitudeTransform()
    trans_y.setAttitude(osg.Quat(-np.pi/2., osg.Vec3(1.,0.,0.)))
    trans_y.addChild(geo_y)
    # create the group and addChild trans_x and y, z didn't need transform
    geo_frame = osg.Group()
    geo_frame.addChild(trans_x)
    geo_frame.addChild(trans_y)
    geo_frame.addChild(geo_z)
    return geo_frame
    
    
def draw_frame(pose=None, text=None, gen_frame=None, scale = 1.):
    """Draw the arrows and label of a frame.
    """
    frame = osg.MatrixTransform()   #create frame node
    # set position
    if pose is None:
        pose = np.eye(4)
    frame.setMatrix(pose_2_mat(pose))
    if gen_frame is None:
        gen_frame = create_generic_frame()
    if text is not None:
        #create the text geode and add it as frame child
        frame_text = osgText.Text()
        frame_text.setCharacterSize(scale)
        frame_text.setText(str(text))
        frame_text.setDrawMode(osgText.Text.TEXT | osgText.Text.BOUNDINGBOX)
        frame_text.setAlignment(osgText.Text.CENTER_TOP)
        #frame_text.setAxisAlignment(osgText.Text.SCREEN)   # un-comment if want HUD version
        geo_text = osg.Geode()
        geo_text.addDrawable(frame_text)
        frame.addChild(gen_frame) #add childs
        frame.addChild(geo_text)  #
    return frame
    
    
def draw_link(start, end, scale, color):
    z = np.array([0.,0.,1.])
    v = np.array([end[0,3]-start[0,3], end[1,3]-start[1,3], end[2,3]-start[2,3]])
    length = np.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if length != 0.:
        v = v/length
        q = np.cross(z,v)
        sin_theta = np.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2])
        cos_theta = np.dot(z,v)
        theta = np.arctan2(sin_theta, cos_theta)
        # create the cylinder
        cyl = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,length/2), 0.1*scale, length))
        cyl.setColor(osg.Vec4(color[0], color[1], color[2], 1.))
        geo = osg.Geode()
        geo.addDrawable( cyl )
        link = osg.PositionAttitudeTransform()
        link.setPosition(osg.Vec3d(start[0,3],start[1,3],start[2,3]))
        if sin_theta == 0.:
            link.setAttitude(osg.Quat(theta, osg.Vec3d(1.,0.,0.)))
        else:
            link.setAttitude(osg.Quat(theta, osg.Vec3d(q[0], q[1], q[2])))
        link.addChild(geo)
        return link
    else:
        return None    

    
    
    
    
class World(visu.World):
    """ A drawable version of rigidmotion.World
    """
    def __init__(self, world, scale):
        self._world = world
        self._scale = scale
        self.bodies = []
        self.COI = np.array([0., 0., 0.])
        self._root = osg.Group()    # we save the root node of our osg
        # create the bodies and save them
        gen_frame = create_generic_frame(self._scale)
        self._color_set = [(1,1,1), (1,0,0), (0,1,0), (0,0,1)]
        i=0
        for b in self._world.bodies:
            color = self._color_set[i%len(self._color_set)]
            self.COI = self.COI + self.add_body(b, scale, color, gen_frame)
            i+=1
        self.COI = self.COI/len(self._world.bodies)
        # create the osg viewer
        self.viewer = osgViewer.Viewer()
        self.viewer.setUpViewInWindow(100,100, 800, 600)
        manipulator = osgGA.TrackballManipulator()
        self.viewer.setCameraManipulator(manipulator)
        manipulator.setHomePosition(osg.Vec3d(self.COI[0]+25*scale, self.COI[1]+25*scale, self.COI[2]+25*scale),
                                    osg.Vec3d(self.COI[0]  , self.COI[1], self.COI[2]),
                                    osg.Vec3d(0,1,0))
        self.viewer.home()
        self.viewer.setSceneData(self._root) # we give the root_node to show
        
        
    def add_body(self, added_body, scale, color, gen_frame):
        new_vbody = Body(added_body, scale, color, gen_frame)
        self._root.addChild(new_vbody.body_node)
        self.bodies.append(new_vbody)
        return new_vbody.COI


    def update(self, showFrames = None, showLinks = None):
        for b in self.bodies:
            b.update(showFrames, showLinks)
# end of class World()





class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def __init__(self, body, scale=1., color=(1,1,1), gen_frame=None):
        self._body = body
        self.frames = []
        self.links = []
        self._color = color
        self._scale = scale
        self.COI = np.array([0., 0., 0.])
        # create the osg body sub tree
        self.body_node = osg.MatrixTransform()
        self.switcher = osg.Switch()
        self.body_node.addChild(self.switcher)
        # create trhe groups ...
        self.group_frames = osg.Group()
        self.group_links = osg.Group()
        self.group_shapes = osg.Group()
        self.switcher.addChild(self.group_frames) # ... and link them with the tree
        self.switcher.addChild(self.group_links)
        self.switcher.addChild(self.group_shapes)
        if gen_frame is None:
            gen_frame = create_generic_frame()
        self.draw_body(gen_frame)
        
    def draw_body(self, gen_frame):
        # set the matrixTransform
        self.body_node.setMatrix(pose_2_mat(self._body.pose))
        self.COI = self._body.pose[0:3,3]
        for f in self._body.frames:
            nf = draw_frame(f.bpose, f.name, gen_frame, self._scale)
            self.group_frames.addChild(nf)
            self.frames.append(nf)
            lf = draw_link(np.eye(4), f.bpose, self._scale, self._color)
            if lf is not None:
                self.group_links.addChild(lf)
                self.links.append(lf)

    def update(self, showFrames, showLinks):
        pose = self._body.pose
        self.body_node.setMatrix(pose_2_mat(pose))
        if showFrames is not None:
            self.switcher.setChildValue(self.group_frames, showFrames)
        if showLinks is not None:
            self.switcher.setChildValue(self.group_links, showLinks)
# end of class Body()





if __name__=='__main__':
    # testing!
    if 0:       # choose between triplehinge and human36
        from worldfactory import triplehinge
        w = triplehinge()
    else:
        import human36
        (w, bd, tags) = human36.human36() # or also: "w = human36.human36()[0]
        
    w.update_geometric()

    import visu_osg
    vw = visu_osg.World(w, 0.1)
    t = 0.
    vw.viewer.realize()
    while(not(vw.viewer.done())):
        t+=1/800.
        if 0: #choose witch robot to control: 1=3hinge; 0=human36
            w.joints[0].gpos=[t]
            w.joints[1].gpos=[t]
            w.joints[2].gpos=[t]
        else:
            #w.joints[0].gpos=np.array([[1,0,0,t],[0,1,0,t],[0,0,1,t],[0,0,0,1]])
            w.joints[1].gpos=[t, t, t]
            w.joints[2].gpos=[t]
            w.joints[3].gpos=[t, t]
            
        w.update_geometric()
        vw.update(True, True) #(showframe, showBody)
        vw.viewer.frame()
        
        
