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
from numpy import pi, arctan2, array, dot, cross, sqrt, eye, cos, sin
import shapes
import core
from misc import NamedObject
import homogeneousmatrix as Hg

def draw_frame(scale=1.):
    # WARN: we create the leaves and go towards the trunk of the osg tree
    # create the x cylinder
    cyl_x = osg.ShapeDrawable(osg.Cylinder(
        osg.Vec3(0.,0.,scale/2.), 0.05*scale, scale))
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

def pose2mat(pose):
    """Convert an homogeneous transform matrix from python to osg. The
    conversion handle the transposition required by osg.
    """
    m = osg.Matrixd()
    m.set(pose[0,0], pose[1,0], pose[2,0], pose[3,0],
          pose[0,1], pose[1,1], pose[2,1], pose[3,1],
          pose[0,2], pose[1,2], pose[2,2], pose[3,2],
          pose[0,3], pose[1,3], pose[2,3], pose[3,3],
          )
    return m

def draw_line(start, end, scale=1, color=None):
    v = end-start
    length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if length != 0.:
        v = v/length
        # create the cylinder
        radius = 0.04*scale
        cyl = osg.ShapeDrawable(
            osg.Cylinder(osg.Vec3(0., 0., length/2), radius, length))
        if color is not None:
            cyl.setColor(color)
        geo = osg.Geode()
        geo.addDrawable(cyl)
        line = osg.PositionAttitudeTransform()
        line.setPosition(osg.Vec3d(start[0], start[1], start[2]))
        z = array([0.,0.,1.])
        q = cross(z, v)
        sin_theta = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2])
        cos_theta = dot(z, v)
        theta = arctan2(sin_theta, cos_theta)
        if sin_theta == 0.:
            line.setAttitude(osg.Quat(theta, osg.Vec3d(1.,0.,0.)))
        else:
            line.setAttitude(osg.Quat(theta, osg.Vec3d(q[0], q[1], q[2])))
        line.addChild(geo)
        return line
    else:
        return None
        

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
        # we create the generic frame node and we will 
        # re-use it for every frame created
        self._generic_frame = draw_frame(scale=self.scale)
        self.alpha = 0.8
        if body_palette is None:
            self.body_palette = [
                (1,0,0),
                (0,1,0),
                (0,0,1),
                (1,1,0),
                (0,1,1)]
        self.body_colors = {}


    def choose_color(self, body):
        if body in self.body_colors:
            color = self.body_colors[obj.body]
        else:
            try:
                c = self.body_palette.pop()
                color = osg.Vec4(c[0], c[1], c[2], self.alpha)
            except IndexError:
                color = osg.Vec4(1, 1 ,1 , self.alpha)
        return color


    def convert(self, obj, parent=None):
        """Given an arboris obect ``obj`` as input, this method
        return a dictionnary of osg nodes for the rendering of 
        ``obj``. I also returns a dictionary of osg switches, in
        order to make it possible to disable the rendering of some nodes.
        Each node is the child of its corresponding switch.

        If the optional ``parent`` argument is given, the switches are its
        children

        for instance, if ``obj`` is an instance of 
        :class:`arboris.core.SubFrame`, and ``parent`` is given, it will 
        result in the following tree::

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

        if isinstance(obj, NamedObject) and (obj.name is not None):
            switches['name'] = osg.Switch()
            nodes['name'] = draw_text(obj.name, self.scale)

        if isinstance(obj, core.Frame):
            switches['frame'] = osg.Switch()
            nodes['frame'] = self._generic_frame

        if isinstance(obj, core.SubFrame):
            color = self.choose_color(obj.body)
            nl = draw_line((0,0,0), 
                           -dot(obj.bpose[0:3,0:3].T, obj.bpose[0:3,3]), 
                                self.scale, color)
            if nl is not None:
                switches['link'] = osg.Switch()
                nodes['link'] = nl

        if isinstance(obj, core.Shape):
            color = self.choose_color(obj.frame.body)

            if isinstance(obj, shapes.Sphere):
                shape = osg.ShapeDrawable(osg.Sphere(osg.Vec3(0.,0.,0.), 
                                                     obj.radius))

            elif isinstance(obj, shapes.Box):
                shape = osg.ShapeDrawable(osg.Box(osg.Vec3(0.,0.,0.), 
                                                  obj.lengths[0], 
                                                  obj.lengths[1], 
                                                  obj.lengths[2]))

            elif isinstance(obj, shapes.Cylinder):
                shape = osg.ShapeDrawable(osg.Cylinder(osg.Vec3(0.,0.,0.), 
                                                       obj.radius, 
                                                       obj.length))

            else:
                raise ValueError("Undrawable shape")

            shape.setColor(color)
            nodes['shape'] = osg.Geode()
            nodes['shape'].addDrawable(shape)
            switches['shape'] = osg.Switch()

        for key in nodes.iterkeys():
            switches[key].addChild(nodes[key])
        if parent is not None:
            for switch in switches.itervalues():
                parent.addChild(switch)
        return (nodes, switches)

class DrawableWorld(core.World):

    def __init__(self, world=None, factory=None, *positional_args, 
                 **keyword_args):
        if world is None:
            core.World.__init__(self,*positional_args, **keyword_args)
            self.update_geometric()
        else:
            raise NotImplemented
        self.drawer = WorldDrawer(self, factory)

        if factory is None:
            self.factory = NodeFactory()
        else:
            self.factory = factory

    def register(self, obj):
        core.World.register(self, obj)
        self.drawer.register(obj, self.factory)

    def init_graphic(self, *positional_args, **keyword_args):
        self.viewer = self.drawer.init_viewer(*positional_args, **keyword_args)
        self.viewer.realize()

    def update_graphic(self):
        self.drawer.update()
        self.viewer.frame()

    def graphic_is_done(self):
        return self.viewer.done()

class WorldDrawer(object):
    """
    
    """
    def __init__(self, world, factory=None):

        if factory is None:
            factory = NodeFactory()

        self.world = world
        self.root = osg.Group()
        # enable the transparency/alpha
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self.root.setStateSet(blend)
        self.transforms = {}
        self.nodes =  {}
        self.switches = {}

        for obj in self.world.iterbodies():
            self.register(obj, factory)

        for obj in self.world.itersubframes():
            self.register(obj, factory)

        for obj in self.world.itershapes():
            self.register(obj, factory)

    def register(self, obj, factory):
        if isinstance(obj, core.Frame):
            if obj in self.transforms:
                pass
            else:
                t = osg.MatrixTransform()
                self.transforms[obj] = t

                if isinstance(obj, core.Body):
                    self.root.addChild(t)
                elif isinstance(obj, core.SubFrame):
                    self.transforms[obj.body].addChild(t)
                else:
                    raise NotImplemented()
                (nodes, switches) = factory.convert(obj, t)
                self.nodes[obj] = nodes
                self.switches[obj] = switches

        elif isinstance(obj, core.Shape):
            if obj in self.nodes or obj in self.switches:
                pass
            else:
                (nodes, switches) = factory.convert(obj, 
                                                    self.transforms[obj.frame])
                self.nodes[obj] = nodes
                self.switches[obj] = switches

        elif isinstance(obj, core.Joint):
            pass

        else:
            raise ValueError(obj)

    def update(self):
        for obj in self.world.itersubframes():
            self.transforms[obj].setMatrix(pose2mat(obj.bpose))
        for obj in self.world.iterbodies():
            self.transforms[obj].setMatrix(pose2mat(obj.pose))

    def switch(self, nodetype, on=True):
        if on:
            for sw in self.switches.itervalues():
                if nodetype in sw:
                    sw[nodetype].setAllChildrenOn()
        else:
            for sw in self.switches.itervalues():
                if nodetype in sw:
                    sw[nodetype].setAllChildrenOff()


    def init_viewer(self, fullscreen=False, window=(0,0,800,600), coi=None, 
                    camera=None, up=(0,1,0)):

        # create the osg viewer
        viewer = osgViewer.Viewer()
        viewer.setSceneData(self.root)
        manipulator = osgGA.TrackballManipulator()
        viewer.setCameraManipulator(manipulator)
        #check if COI and cam relative distance are set
        if coi is None:
            coi = array([0., 0., 0.])
            nframes = 0
            for f in self.world.iterframes():
                coi = coi + f.pose[0:3,3]
                nframes +=1
                coi = coi/nframes
        if camera is None:
            camera = [3, 3, 3]
    
        #we set the position of the camera/view
        manipulator.setHomePosition(
            osg.Vec3d(coi[0]+camera[0], coi[1]+camera[1], coi[2]+camera[2]),
            osg.Vec3d(coi[0], coi[1], coi[2]),
            osg.Vec3d(up[0],up[1],up[2]))     
        #check if we want to see simulation in a window
        if fullscreen is False:
            viewer.setUpViewInWindow(window[0], window[1], 
                                     window[2], window[3])
        viewer.home()
        return viewer

