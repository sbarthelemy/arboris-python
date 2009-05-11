# coding=utf-8
"""
Visualization of a simulation

This module is based on openscenegraph (osg) 2.6 python wrappers.

A scene graph is a data structure that arranges the logical and spatial
representation of a graphical scene. It consists of a collection of *nodes* in 
a tree or graph structure. In principle node may have many children but only a single
parent, with the effect of a parent apparent to all its child nodes. For
instance, a geometrical transformation matrix node would move
all its children.

In osg, a single node may be shared between several parents, for instance when
the same geometry is displayed several times simulatneously (this saves memory).

Classes:
    - Node
    - Group
    - Geode (*geometry node*)
    - MatrixTransform
    - Switch

A geode can contain:
    - DrawableShape
    - Geometry
    - 

Viewer—The Viewer class can manage multiple synchronized cameras to
render a single view spanning multiple monitors. Viewer creates its own
window(s) and graphics context(s) based on the underlying graphics system
capabilities, so a single Viewer-based application executable runs on single or
multiple display systems.

TODO: try on windows and fix the import.

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from OpenSceneGraph import osg, osgDB, osgGA, osgViewer, osgText # on linux
#import osg, osgDB, osgGA, osgViewer, osgText                    # on windows
import numpy as np
import visu
import arboris
import misc

#we create the frame node and we will re-use it for every frame created
def create_generic_frame(scale=.1):
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
    frame = osg.Group()
    frame.addChild(trans_x)
    frame.addChild(trans_y)
    frame.addChild(geo_z)
    return frame

generic_frame = create_generic_frame()

class Color(visu.Color):
    """
    """
    def __init__(self, var1, var2=1.):
        visu.Color.__init__(self, var1, var2)
        
    def get(self):
        return osg.Vec4(self.rgba[0], self.rgba[1], self.rgba[2], self.rgba[3])
        


def pose2mat(pose):
    """ Convert an homogeneous transform matrix from python to osg
    """
    m = osg.Matrixd()
    m.set(pose[0,0], pose[1,0], pose[2,0], pose[3,0],
          pose[0,1], pose[1,1], pose[2,1], pose[3,1],
          pose[0,2], pose[1,2], pose[2,2], pose[3,2],
          pose[0,3], pose[1,3], pose[2,3], pose[3,3],
          ) # here we do the f***ing transposition: beware of error
    return m

def rot2quat(rot):
    #TODO: all the computation to transform rot in quat
    raise NotImplemented()
    q = osg.quat(1.,osg.Vec3d(0.,0.,0.))
    return q


def create_generic_arrow(scale=1., color=Color('white')):
    cone = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,scale), 0.05*scale, scale))
    cone.setColor(color.get())
    geo_cone = osg.Geode()
    geo_cone.addDrawable(cone)
    return geo_cone
    
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
        cyl.setColor(color.get())
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
        
def draw_shape(shape=None, scale=1.):
    if  shape.name is 'sphere':
        shp = osg.ShapeDrawable(osg.Sphere(osg.Vec3(0.,0.,0.),shape.dim[0]))
    elif shape.name is 'box':
        shp = osg.ShapeDrawable(osg.Box(osg.Vec3(0.,0.,0.),shape.dim[0],shape.dim[1],shape.dim[2]))
    elif shape.name is 'cylinder':
        shp = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,0.),shape.dim[0],shape.dim[1]))
    elif shape.name is 'point':
        shp = osg.ShapeDrawable(osg.Sphere(osg.Vec3(0.,0.,0.),0.01*scale))
    
    shp.setColor(Color('brown', 0.3).get())
    geo = osg.Geode()
    geo.addDrawable(shp)
    shape_node = osg.MatrixTransform()
    shape_node.setMatrix(pos2mat(shape.frame))
    return geo
    
    
    
    
    
class World(visu.World):
    """ A drawable version of rigidmotion.World
    """
    def __init__(self, world, scale=1.):
        visu.World.__init__(self, world, scale)
        
        self._root = osg.Group()    # we save the root node of our osg
        # enable the transparency/alpha
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self._root.setStateSet(blend)
        # create the osg viewer
        self.viewer = osgViewer.Viewer()
        self.viewer.setSceneData(self._root) # we give the root_node to show
        self.manipulator = osgGA.TrackballManipulator()
        self.viewer.setCameraManipulator(self.manipulator)

    def add_body(self, added_body, color, gen_frame):
        new_vbody = Body(added_body, self._scale, color, gen_frame)
        self._root.addChild(new_vbody.body_node)
        self.bodies.append(new_vbody)
        
    def add_wrench(self, added_wrench, gen_force_arrow, gen_moment_arrow):
        new_wrench = Wrench(added_wrench, self._scale, gen_force_arrow, gen_moment_arrow)
        self._root.addChild(new_wrench)
        self.wrenches.append(new_wrench)

    def update(self, showFrames = None, showLinks = None):
        for b in self.bodies:
            b.update(showFrames, showLinks)
# end of class World()





class Body(visu.Body):
    """ A drawable version of rigidmotion.Body
    """
    def __init__(self, body, scale=1., color=Color('white'), gen_frame=None):
        visu.Body.__init__(self, body, scale, color)
        
        # create the osg body sub tree
        self.body_node = osg.MatrixTransform()
        self.switcher = osg.Switch()
        self.body_node.addChild(self.switcher)
        # create the groups ...
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
        self.body_node.setMatrix(pose2mat(self._body.pose))
        #TODO: repair!
        #for f in self._body.frames:
        f = self._body
        nf = draw_frame(f.bpose, f.name, self._scale)
        self.group_frames.addChild(nf)
        self.frames.append(nf)
        nl = draw_link(np.eye(4), f.bpose, self._scale, self._color)
        if nl is not None:
            self.group_links.addChild(nl)
            self.links.append(nl)
        #for s in self._body.shapes:
        #    ns = draw_shape(s)
        #    self.group_shapes.addChild(ns)
        #    self.shapes.append(ns)

    def update(self, showFrames, showLinks):
        pose = self._body.pose
        self.body_node.setMatrix(pose2mat(pose))
        if showFrames is not None:
            self.switcher.setChildValue(self.group_frames, showFrames)
        if showLinks is not None:
            self.switcher.setChildValue(self.group_links, showLinks)
# end of class Body()





class Wrench(object):
    #TODO: make the Abstract Class in visu.py
    """
    """
    def __init__(self, wrench, scale=1., gen_force_arrow=None, gen_moment_arrow=None):
        self._wrench = wrench
        self._scale = scale
        self.force_node  = osg.PositionAttitudeTransform()
        self.moment_node = osg.PositionAttitudeTransform()
        self.force_node.addChild(gen_force_arrow)
        self.moment_node.addChild(gen_moment_arrow)
        self.wrench_node = osg.Switch()
        self.wrench_node.addChild(self.force_node)
        self.wrench_node.addChild(self.moment_node)
        self.wrench_node.setAllChildrenOff()
        
        def update(self, show):
            if self._wrench.is_active is True:
                self.wrench_node.setAllChildrenOn()
                #TODO: make the transformation!!!
                #self.force_node.setPosition(pose2mat(self._wrench.pose))
                #self.force_node.setAttitude(rot2quat(self._wrench.pose))
                #self.force_node.setScale(osg.Vec3d(self._wrench.force, self._wrench.force, self._wrench.force))
                #self.moment_node.setPosition(pose2mat(self._wrench.pose))
                #self.moment_node.setAttitude(rot2quat(self._wrench.pose))
                #self.moment_node.setScale(osg.Vec3d(self._wrench.moment, self._wrench.moment, self._wrench.moment))
            else:
                self.wrench_node.setAllChildrenOff()
# end of class Wrench


# World Factory
def World_Factory(arb_world, scale=1., color_set=None, 
                  windowed=(0,0,800,600), COI=None, cam=None, up = (0,1,0)):
    #create the visual world
    v_world = World(arb_world, scale)
    #check if color_set exist
    if color_set is None:
        color_set = [Color('white'), Color('red'), Color('green'), Color('blue')]
        
    # create the bodies and save them
    i=0
    gen_frame = create_generic_frame(scale)
    for b in arb_world.bodies:
        color = color_set[i%len(color_set)]
        v_world.add_body(b, color, gen_frame)
        i+=1
    
    #create the wrench and save them
    gen_force = create_generic_arrow(scale, Color('green'))
    gen_moment = create_generic_arrow(scale, Color('red'))
    ##### #####
    if 0: # delete it to test wrenches
        for w in arb_world.wrenches:
            v_world.add_wrench(w, gen_force, gen_moment)
        
        
    #check if COI and cam relative distance are set
    if COI is None:
        COI = np.array([0., 0., 0.])
        for b in v_world._world.bodies:
            COI = COI + b.pose[0:3,3]
        COI = COI/len(v_world._world.bodies)
    if cam is None:
        cam = [25*scale, 25*scale, 25*scale]
    
    #we set the position of the beginning camera/view
    v_world.manipulator.setHomePosition(osg.Vec3d(COI[0]+cam[0], COI[1]+cam[1], COI[2]+cam[2]),
                                    osg.Vec3d(COI[0], COI[1], COI[2]),
                                    osg.Vec3d(up[0],up[1],up[2]))     
    v_world.viewer.home()

    #check if we want to see simulation in a window
    if (windowed is not False) & (windowed is not None):
        v_world.viewer.setUpViewInWindow(windowed[0], windowed[1], 
                                         windowed[2], windowed[3])
        
    return v_world
# end of World Factory


####### SEB #######

def draw_frame(pose=None, label=None, scale=1.):
    """Draw the arrows and label of a coordinate frame.
    """
    frame = osg.MatrixTransform()   #create frame node
    if pose is None:
        pose = np.eye(4)
    frame.setMatrix(pose2mat(pose))
    if label is not None:
        frame.addChild(draw_text(label))
    frame.addChild(generic_frame)
    return frame

def draw_text(label, scale=1.):
    """create a text geode
    """
    text = osgText.Text()
    text.setCharacterSize(scale)
    text.setText(str(label))
    text.setDrawMode(osgText.Text.TEXT | osgText.Text.BOUNDINGBOX)
    text.setAlignment(osgText.Text.CENTER_TOP)
    #text.setAxisAlignment(osgText.Text.SCREEN)   # un-comment for HUD version
    geode = osg.Geode()
    geode.addDrawable(text)
    return geode



class NodeFactory(object):

    def convert(self, obj, parent=None):
        
        nodes = {}
        if isinstance(obj, misc.NamedObject) & (obj.name is not None):
            nodes['name'] = draw_text(obj.name)

        if isinstance(obj, arboris.Frame):
            nodes['frame'] = generic_frame

        if isinstance(obj, arboris.SubFrame):
            nl = draw_link(np.eye(4), obj.bpose, scale=1., color=Color('white'))
            if nl is not None:
                nodes['link'] = nl

        if parent is not None:
            for node in nodes.itervalues():
                parent.addChild(node)
        return nodes

class SebWorld(object):
    """
    TODO: merge ``body_nodes`` and ``subframe_nodes`` into ``frame_nodes`` (or
    better, ``transform_nodes``)?
    """
    def __init__(self, world, factory=None):

        if factory is None:
            factory = NodeFactory()

        self.world = world
        self.root_node = osg.Group()
        self.body_nodes = {}
        self.subframe_nodes = {}
        self.drawable_nodes = {'frame':{}, 'name':{}, 'link':{}, 'shape':{}}

        for b in self.world.ground.descendants():
            bn = osg.MatrixTransform()
            self.body_nodes[b] = bn
            self.root_node.addChild(bn)
            nodes = factory.convert(b, bn)
            for key, value in nodes.iteritems():
                self.drawable_nodes[key][b] = nodes[key]
        for sf in self.world._subframes:
            sfn = osg.MatrixTransform()
            self.subframe_nodes[sf] = sfn
            self.body_nodes[sf.body].addChild(sfn)
            nodes = factory.convert(sf, sfn)
            for key, value in nodes.iteritems():
                self.drawable_nodes[key][b] = nodes[key]

        self.frame_nodes = self.subframe_nodes.copy()
        self.frame_nodes.update(self.body_nodes)
        for sh in self.world._shapes:
            self.drawable_nodes['shape'][sh] = \
                factory.convert(sh, self.frame_nodes[sh.frame])
        self.update()

        # testing switch
        self.switch = osg.Switch()
        for f in self.drawable_nodes['name'].itervalues():
            #self.switch.addChild(f, True)
            self.switch.addChild(f, False)


    def update(self):
        for b in self.world.ground.descendants():
            self.body_nodes[b].setMatrix(pose2mat(b.pose))

        for sf in self.world._subframes:
            self.subframe_nodes[sf].setMatrix(pose2mat(sf.bpose))

class SebOsgFactory(object):

    def init(self, world):

        # enable the transparency/alpha
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        world.root_node.setStateSet(blend)
        # create the osg viewer
        viewer = osgViewer.Viewer()
        viewer.setSceneData(world.root_node)
        manipulator = osgGA.TrackballManipulator()
        viewer.setCameraManipulator(manipulator)
        # bodies
        viewer.home()
        viewer.realize()
        t=0.
        while(not(viewer.done())):
            t+=1/800.
            world.world.joints[0].gpos=[t]
            world.world.joints[1].gpos=[t]
            world.world.joints[2].gpos=[t]
            world.world.update_geometric()
            world.update()
            viewer.frame()

if __name__ == '__main__':
    # testing!
    if 0:       # choose between triplehinge and human36
        from worldfactory import triplehinge
        w = triplehinge()
    else:
        import human36
        (w, bd, tags) = human36.human36() # or also: "w = human36.human36()[0]
        
    w.update_geometric()

    import visu_osg
    colors = [Color('velvet'), Color([1.,0.,0.,.7]), Color((0.,1.,0.,), .4), Color('brown',0.2)]
    #vw = visu_osg.World_Factory(w, 0.1, colors, (200, 100, 600, 600), (0,0,0), (2,2,2), (0,1,1))
    vw = visu_osg.World_Factory(w, 0.1, colors)
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
        
        
