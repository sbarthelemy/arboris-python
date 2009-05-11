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

The Viewer class can manage multiple synchronized cameras to
render a single view spanning multiple monitors. Viewer creates its own
window(s) and graphics context(s) based on the underlying graphics system
capabilities, so a single Viewer-based application executable runs on single or
multiple display systems.

TODO: try on windows

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from OpenSceneGraph import osg, osgDB, osgGA, osgViewer, osgText
from numpy import pi, arctan2, array, dot, cross, sqrt, eye
import visu
import arboris
from misc import Color, NamedObject
import homogeneousmatrix as Hg

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
    trans_x.setAttitude(osg.Quat(pi/2., osg.Vec3(0.,1.,0.)))
    trans_x.addChild(geo_x)
    trans_y = osg.PositionAttitudeTransform()
    trans_y.setAttitude(osg.Quat(-pi/2., osg.Vec3(1.,0.,0.)))
    trans_y.addChild(geo_y)
    # create the group and addChild trans_x and y, z didn't need transform
    frame = osg.Group()
    frame.addChild(trans_x)
    frame.addChild(trans_y)
    frame.addChild(geo_z)
    return frame

generic_frame = create_generic_frame()

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

def rgba2vec4(rgba):
    return osg.Vec4(rgba[0], rgba[1], rgba[2], rgba[3])

def create_generic_arrow(scale=1., color=None):
    cone = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,scale), 0.05*scale, scale))
    if color is not None:
        cone.setColor(rgba2vec4(color.rgba))
    geo_cone = osg.Geode()
    geo_cone.addDrawable(cone)
    return geo_cone
    
def draw_link(start, end, scale, color=None):
    z = array([0.,0.,1.])
    v = array([end[0,3]-start[0,3], end[1,3]-start[1,3], end[2,3]-start[2,3]])
    length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if length != 0.:
        v = v/length
        q = cross(z,v)
        sin_theta = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2])
        cos_theta = dot(z,v)
        theta = arctan2(sin_theta, cos_theta)
        # create the cylinder
        cyl = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,length/2), 0.1*scale, length))
        if color is not None:
            cyl.setColor(rgba2vec4(color.rgba))
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
    
    shp.setColor(osg.Vec4(1,1,0,0.3))
    geo = osg.Geode()
    geo.addDrawable(shp)
    shape_node = osg.MatrixTransform()
    shape_node.setMatrix(pos2mat(shape.frame))
    return geo
    
    
    
    
    
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


def draw_frame(pose=None, label=None, scale=1.):
    """Draw the arrows and label of a coordinate frame.
    """
    frame = osg.MatrixTransform()   #create frame node
    if pose is None:
        pose = eye(4)
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
    """
    Given arboris objects, th
    """
    def __init__(self, scale=1., body_palette=None):
    
        self.scale = scale
        if body_palette is None:
            self.body_palette = [Color('red'), Color('green'), Color('blue')]
        self.body_colors = {}


    def convert(self, obj, parent=None):
        """Given an arboris obect ``obj`` as input, this method
        return a dictionnary of osg nodes for the rendering of 
        ``obj``. I also returns a dictionary of osg switches, in
        order to make it possible to disable the rendering of some nodes.
        Each node is the child of its corresponding switch.

        If the optional ``parent`` argument is given, the switches are its
        children

        for instance, if ``obj`` is an instance of :class:`arboris.SubFrame`, 
        and ``parent`` is given, it will result in the following tree::

            parent
              |
              +-----------------+------------------+
              |                 |                  |
            switches['name']  switches['frame']  switches['link']
              |                 |                  |
            nodes['name']     nodes['frame']     nodes['link']

        """
        
        nodes = {}
        switches = {}
        if isinstance(obj, NamedObject) & (obj.name is not None):
            switches['name'] = osg.Switch()
            nodes['name'] = draw_text(obj.name, self.scale)

        if isinstance(obj, arboris.Frame):
            switches['frame'] = osg.Switch()
            nodes['frame'] = generic_frame

        if isinstance(obj, arboris.SubFrame):
            if obj.body in self.body_colors:
                color = self.body_colors[obj.body]
            else:
                try:
                    color = self.body_palette.pop()
                except IndexError:
                    color = Color('white')
                self.body_colors[obj.body] = color
            nl = draw_link(Hg.inv(obj.bpose), eye(4), self.scale, color)
            if nl is not None:
                switches['link'] = osg.Switch()
                nodes['link'] = nl

        for key in nodes.iterkeys():
            switches[key].addChild(nodes[key])
        if parent is not None:
            for switch in switches.itervalues():
                parent.addChild(switch)
        return (nodes, switches)

class World(object):
    """
    TODO: merge ``body_nodes`` and ``subframe_nodes`` into ``frame_nodes`` (or
    better, ``transform_nodes``)?
    """
    def __init__(self, world, factory=None):

        if factory is None:
            factory = NodeFactory()

        self.world = world
        self.root_node = osg.Group()
        # enable the transparency/alpha
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self.root_node.setStateSet(blend)
        self.transforms = {}
        self.nodes = {'frame':{}, 'name':{}, 'link':{}, 'shape':{}}
        self.switches = {'frame':{}, 'name':{}, 'link':{}, 'shape':{}}

        for b in self.world.iterbodies():
            bn = osg.MatrixTransform()
            self.transforms[b] = bn
            self.root_node.addChild(bn)
            (nodes, switches) = factory.convert(b, bn)
            for key, value in nodes.iteritems():
                self.nodes[key][b] = value
            for key, value in switches.iteritems():
                self.switches[key][b] = value

        for sf in self.world._subframes:
            sfn = osg.MatrixTransform()
            self.transforms[sf] = sfn
            self.transforms[sf.body].addChild(sfn)
            (nodes, switches) = factory.convert(sf, sfn)
            for key, value in nodes.iteritems():
                self.nodes[key][sf] = value
            for key, value in switches.iteritems():
                self.switches[key][sf] = value

        for sh in self.world._shapes:
            self.drawable_nodes['shape'][sh] = \
                factory.convert(sh, self.transforms[sh.frame])
        self.update()

    def update(self):
        for b in self.world.iterbodies():
            self.transforms[b].setMatrix(pose2mat(b.pose))

        for sf in self.world._subframes:
            self.transforms[sf].setMatrix(pose2mat(sf.bpose))
   
    def switch(self, nodetype, on=True):
        if on:
            for sw in self.switches[nodetype].itervalues():
                sw.setAllChildrenOn()
        else:
            for sw in self.switches[nodetype].itervalues():
                sw.setAllChildrenOff()

    def init_viewer(self, windowed = (0,0,800,600), COI=None, cam=None, up=(0,1,0)):

        # create the osg viewer
        viewer = osgViewer.Viewer()
        viewer.setSceneData(self.root_node)
        manipulator = osgGA.TrackballManipulator()
        viewer.setCameraManipulator(manipulator)
        #check if COI and cam relative distance are set
        if COI is None:
            COI = array([0., 0., 0.])
        nbodies = 0
        for b in self.world.iterbodies():
            COI = COI + b.pose[0:3,3]
            nbodies +=1
        COI = COI/nbodies
        if cam is None:
            cam = [3, 3, 3]
    
        #we set the position of the beginning camera/view
        manipulator.setHomePosition(osg.Vec3d(COI[0]+cam[0], COI[1]+cam[1], COI[2]+cam[2]),
                                    osg.Vec3d(COI[0], COI[1], COI[2]),
                                    osg.Vec3d(up[0],up[1],up[2]))     
        #check if we want to see simulation in a window
        if (windowed is not False) & (windowed is not None):
            viewer.setUpViewInWindow(windowed[0], windowed[1], 
                                     windowed[2], windowed[3])
        viewer.home()
        return viewer

if __name__ == '__main__':
    from visu_osg import NodeFactory, World
    
    test_triplehinge = False
    if test_triplehinge:
        from triplehinge import triplehinge
        w = triplehinge()
    else:
        from human36 import human36
        (w, bd, tags) = human36()
    w.update_geometric()
    
    nf = NodeFactory(scale=.1)

    vw = World(w, nf)
    #vw.switch('name', False)
    viewer = vw.init_viewer()
    viewer.realize()
    t = 0.
    while(not(viewer.done())):
        t+=1/800.
        if test_triplehinge:
            w.joints[0].gpos[0]=t
            w.joints[1].gpos[0]=t
            w.joints[2].gpos[0]=t
        else:
            w.joints[1].gpos=[t, t, t]
            w.joints[2].gpos=[t]
            w.joints[3].gpos=[t, t]
        w.update_geometric()
        vw.update()
        viewer.frame()
        
        
