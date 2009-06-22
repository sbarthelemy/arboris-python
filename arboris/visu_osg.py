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

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
              "Joseph SALINI <joseph.salini@gmail.com>")

from OpenSceneGraph import osg, osgDB, osgGA, osgViewer, osgText
from numpy import pi, arctan2, array, dot, cross, sqrt, eye, cos, sin
import shapes
import core
from misc import NamedObject
import homogeneousmatrix as Hg

import commands
import logging
logging.basicConfig(level=logging.DEBUG)

def com_position(Mb):
    """Principal frame of inertia.

    (Origin is the center of mass, and basis vectors are along principal axis).
    TODO: move this function to somewhere else

    """
    from numpy import linalg, vstack, hstack, zeros, diag
    m = Mb[5,5]
    if m>0:
        r_tilde = Mb[0:3,3:6]/m
        r = array([r_tilde[2,1],r_tilde[0,2],r_tilde[1,0]])
        RSR = Mb[0:3,0:3] + m*dot(r_tilde, r_tilde)
        [S, R] = linalg.eig(RSR)
        if linalg.det(R)<0:
            iI = array([[0,0,1],[0,1,0],[1,0,0]])
            R = dot(R, iI)
            S = dot(iI, dot(S, iI))
        R = eye(3)
        H_b_com = vstack((hstack((R  , r.reshape(3,1))), array([0,0,0,1])))
        Mg = zeros((6,6))
        Mg[0:3,0:3] = diag(S)
        Mg[3:6,3:6] = m*eye(3)
        return [H_b_com, Mg]
    else:
        return [eye(4), zeros((6,6))]



def draw_frame(length=1., radius=0.05, alpha=1.):
    """
    create a pointer to an osg node that represents a frame
    this pointer with be used to draw every frame
    >>> generic_frame = draw_frame(.5, .8)
    """
    # WARN: we create the leaves and go towards the trunk of the osg tree
    # create the x cylinder
    cyl_x = osg.ShapeDrawable(osg.Cylinder(
        osg.Vec3(0.,0.,length/2.), radius, length))
    cyl_y = osg.ShapeDrawable(cyl_x)
    cyl_z = osg.ShapeDrawable(cyl_x)
    cyl_x.setColor(osg.Vec4(1.,0.,0.,alpha))
    cyl_y.setColor(osg.Vec4(0.,1.,0.,alpha))
    cyl_z.setColor(osg.Vec4(0.,0.,1.,alpha))
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
    frame = osg.PositionAttitudeTransform()
    frame.addChild(trans_x)
    frame.addChild(trans_y)
    frame.addChild(geo_z)
    return frame

def pose2mat(pose):
    """Convert an homogeneous transform matrix from python to osg. 
    
    The conversion handles the transposition required by osg.

    :param pose: the homogeneous transform matrix.
    :type pose: (4,4)-shaped ndarray

    **Example:**

    >>> mat = array([[ 1.,  2.,  3.,  4.],
    ...              [ 5.,  6.,  7.,  8.],
    ...              [ 9., 10., 11., 12.],
    ...              [13., 14., 15., 16.]])
    >>> osg_mat = pose2mat(mat)
    
    """
    m = osg.Matrixd()
    m.set(pose[0,0], pose[1,0], pose[2,0], pose[3,0],
          pose[0,1], pose[1,1], pose[2,1], pose[3,1],
          pose[0,2], pose[1,2], pose[2,2], pose[3,2],
          pose[0,3], pose[1,3], pose[2,3], pose[3,3],
          )
    return m


def draw_line(start, end, radius=0.04, color=None):
    """Draw a line between two points.

    The line is rendered as a cylinder.

    :param start: start point
    :type start: (3,)-shaped ndarray
    :param end: end point
    :type end: (3,)-shaped ndarray
    :param radius: the cylinder radius is 0.04*scale
    :type radius: float
    :param color: the line color 
    :type color: osg.Vec4
    :rtype: osg.Transform

    **Example:**

    >>> draw_line(array((1.,2.,3.)), array((4.,5.,6.)), radius=.5) #doctest: +ELLIPSIS
    <OpenSceneGraph.osg.PositionAttitudeTransform; proxy of <Swig Object of type 'osg::PositionAttitudeTransform *' at 0x...> >

    TODO: raise an exception when start==end?

    """
    v = end - start
    length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if length != 0.:
        v = v/length
        # create the cylinder
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
        

def draw_text(label, size=1.):
    """Create a text geode.

    :param label: the text
    :type lable: string
    :param scale: a scaling factor
    :type scale: float

    **Example:**

    >>> draw_text('le texte a afficher', .2) #doctest: +ELLIPSIS
    <OpenSceneGraph.osg.Geode; proxy of <Swig Object of type 'osg::Geode *' at 0x...> >
    
    """
    text = osgText.Text()
    text.setCharacterSize(size)
    text.setText(str(label))
    text.setDrawMode(osgText.Text.TEXT | osgText.Text.BOUNDINGBOX)
    text.setAlignment(osgText.Text.CENTER_TOP)
    #text.setAxisAlignment(osgText.Text.SCREEN)   # un-comment for HUD version
    geode = osg.Geode()
    geode.addDrawable(text)
    return geode


def graphic_options(scale=1.):
    body_palette = [
        (1,0,0),
        (0,1,0),
        (0,0,1),
        (1,1,0),
        (0,1,1)]
    options = {
        'frame length': 0.08 * scale,
        'frame radius': 0.005 * scale,
        'frame alpha': 1.,
        'link radius': 0.004 * scale,
        'text size': 0.1 * scale,
        'body palette': body_palette}
    return options


class WorldDrawer(object):
    """
    
    """

    def __init__(self, world=None, scale=1., options=None):
        if options is None:
            self._options = graphic_options(scale)
        else:
            self._options = options
        self.root = osg.Group()
        # enable the transparency/alpha
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self.root.setStateSet(blend)
        
        self._generic_frame = draw_frame(
            length=self._options['frame length'],
            radius=self._options['frame radius'],
            alpha=self._options['frame alpha'])
        self._body_colors = {}
        self.transforms = {}
        self.switches = {}
        self.is_displayed = {
            'frame': True,
            'link': True,
            'name': True,
            'shape': True,
            'inertia ellipsoid': True}
        if world is not None:
            self.init(world)

    def init(self, world):
        self.world = world
        for obj in self.world.iterbodies():
            self.register(obj)
        for obj in self.world.itersubframes():
            self.register(obj)
        for obj in self.world.itershapes():
            self.register(obj)

    def _choose_color(self, body, alpha=1.):
        if body in self._body_colors:
            color = self._body_colors[body]
        else:
            try:
                c = self._options['body palette'].pop()
                color = osg.Vec4(c[0], c[1], c[2], alpha)
            except IndexError:
                color = osg.Vec4(1., 1., 1., alpha)
        return color

    def register(self, obj):
        """Given an arboris obect ``obj`` as input, this method
        return a dictionnary of OSG nodes for the rendering of 
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

        - ``name`` for any instance of the :class:`arboris.misc.NamedObject` class
        - ``frame`` for objects of the :class:`arboris.core.Frame` class
        - ``link`` lines for skeletton-like view
        - ``shapes`` the basic shapes (:class:`arboris.core.Shape`) used in the 
          simulation for contact detection
        - ``geometry``
        - ``cog``
        - ``mass``
        - 

        """
        # create a transform for the frames (instances of 
        # the Body and Subframe classes)
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
                
        # create other nodes
        if obj in self.switches:
            pass
        else:
            switches = {}
            opts = self._options
            if isinstance(obj, core.SubFrame):
                parent = self.transforms[obj]
                color = self._choose_color(obj.body)
                nl = draw_line((0,0,0),
                               -dot(obj.bpose[0:3,0:3].T, obj.bpose[0:3,3]),
                               opts['link radius'], color)
                switches['link'] = osg.Switch()
                switches['link'].addChild(nl)
            elif isinstance(obj, core.Body):
                parent = self.transforms[obj]
                Mb = obj.mass
                if Mb[5,5] != 0:
                    # MatrixTransform() # position
                    #  |
                    # PositionAttitudeTransform() # global scaling
                    #  |
                    # PositionAttitudeTransform() # ellispoid axis scale
                    #  |
                    # Geode()
                    #  |
                    # Sphere()
                    #
                    [bHg, Mg] = com_position(Mb)
                    shape = osg.ShapeDrawable(
                        osg.Sphere(osg.Vec3(0.,0.,0.), 1))
                    shape.setColor(osg.Vec4(1,1,1,0.5))
                    shape_geo = osg.Geode()
                    shape_geo.addDrawable(shape)
                    scale_node = osg.PositionAttitudeTransform()
                    scale_node.setScale(osg.Vec3d(Mg[0,0], Mg[1,1], Mg[2,2]))
                    scale_node.addChild(shape_geo)
                    gen_scale_node = osg.PositionAttitudeTransform()
                    gen_scale_node.addChild(scale_node)
                    pos_node = osg.MatrixTransform()
                    pos_node.setMatrix(pose2mat(bHg))
                    pos_node.addChild(gen_scale_node)
                    switches['inertia ellipsoid'] = osg.Switch()
                    switches['inertia ellipsoid'].addChild(pos_node)
            elif isinstance(obj, core.Shape):
                parent = self.transforms[obj.frame]
                color = self.choose_color(obj.frame.body)
                if isinstance(obj, shapes.Sphere):
                    shape = osg.ShapeDrawable(
                        osg.Sphere(osg.Vec3(0.,0.,0.), obj.radius))
                elif isinstance(obj, shapes.Box):
                    shape = osg.ShapeDrawable(
                        osg.Box(osg.Vec3(0.,0.,0.), obj.lengths[0], 
                                obj.lengths[1], obj.lengths[2]))
                elif isinstance(obj, shapes.Cylinder):
                    shape = osg.ShapeDrawable(
                        osg.Cylinder(osg.Vec3(0.,0.,0.),
                                     obj.radius, obj.length))
                else:
                    raise NotImplemented("Cannot draw this shape")
                shape.setColor(color)
                switches['shape'] = osg.Switch()
                switches['shape'].addChild(
                    osg.Geode().addDrawable(shape))
            #elif isinstance(obj, str):
                # TODO handle exceptions here
                #parent = self.transforms[obj]
                #nodes['geometry'] = osg.PositionAttitudeTransform()
                #nodes['geometry'].addChild(osgDB.readNodeFile(obj))
                #nodes['geometry'].setScale(osg.Vec3d(.003,.003,.003)) #TODO: DELETE THIS LINE LATER
                #switches['geometry'] = osg.Switch()
            elif isinstance(obj, core.Joint) or \
                isinstance(obj, core.Constraint) or \
                isinstance(obj, core.Controller):
                parent = None
            else:
                raise NotImplemented(obj)

            if isinstance(obj, NamedObject) and (obj.name is not None):
                switches['name'] = osg.Switch()
                switches['name'].addChild(
                    draw_text(obj.name, opts['text size']))
            if isinstance(obj, core.Frame):
                switches['frame'] = osg.Switch()
                switches['frame'].addChild(self._generic_frame)

            if len(switches) != 0:
                for key, val in switches.items():
                    parent.addChild(val)
                    self.switches[obj] = {}
                    self.switches[obj][key] = val

    def update(self):
        for obj in self.world.itersubframes():
            self.transforms[obj].setMatrix(pose2mat(obj.bpose))
        for obj in self.world.iterbodies():
            self.transforms[obj].setMatrix(pose2mat(obj.pose))

    def switch(self, nodetype):
        self.is_displayed[nodetype] = not(self.is_displayed[nodetype])
        for sw in self.switches.itervalues():
            if nodetype in sw:
                if self.is_displayed[nodetype]:
                    sw[nodetype].setAllChildrenOn()
                else:
                    sw[nodetype].setAllChildrenOff()
    
    def rescale(self, coeff):
        #TODO: do the actual rescaling
        #self.scale *= coeff
        #s = osg.Vec3d(self.scale, self.scale, self.scale)
        #self.generic_frame.setScale(s)
        pass


def init_viewer(root, fullscreen=False, window=(0,0,800,600), 
                coi=(0,0,0), camera=(3,3,3), up=(0,1,0)):
        # create the osg viewer:
        viewer = osgViewer.Viewer()
        viewer.setSceneData(root)
        #fill the osgViewer:
        manipulator = osgGA.TrackballManipulator()
        viewer.setCameraManipulator(manipulator)
        #we set the position of the camera/view
        manipulator.setHomePosition(
            osg.Vec3d(coi[0]+camera[0], coi[1]+camera[1], coi[2]+camera[2]),
            osg.Vec3d(coi[0], coi[1], coi[2]),
            osg.Vec3d(up[0], up[1], up[2]))     
        #check if we want to see simulation in a window
        if fullscreen is False:
            if len(window) == 2:
                viewer.setUpViewInWindow(0.,0.,window[0], window[1])
            elif len(window) == 4:
                viewer.setUpViewInWindow(window[0], window[1], 
                                        window[2], window[3])
        viewer.home()
        #kbh = KeyboardHandler(self)
        #viewer.addEventHandler(kbh.__disown__())
        return viewer     


class KeyboardHandler(osgGA.GUIEventHandler):
    base_class = osgGA.GUIEventHandler
    
    def __init__(self, drawer):
        osgGA.GUIEventHandler.__init__(self)
        self._drawer = drawer
        self._logger = logging.getLogger('visu_osg.KeyboardHandler')

    def handle(self, ea, aa, obj, nv):

        def _update_action(action, drawer):
            if action is not None:
                if action == ord('f'):
                    drawer.switch('frame')
                elif action == ord('l'):
                    drawer.switch('link')
                elif action == ord('n'):
                    drawer.switch('name')
                elif action == ord('s'):
                    drawer.switch('shape')
                elif action == ord('i'):
                    drawer.switch('inertia ellipsoid')
                elif action == ord('+'): # + as increasing
                    drawer.rescale(1.25)
                elif action == ord('-'): # - as decreasing
                    drawer.rescale(.8)

        eventtype = ea.getEventType()
        if eventtype == ea.KEYDOWN:
            action = ea.getKey()
            if action < 256:
                self._logger.info('action %d: %s', action, chr(action))
            else:
                self._logger.info('action %d', action)
            _update_action(action, self._drawer)
        return False


class DrawableWorld(core.World):

    def __init__(self, world=None, scale=1., *positional_args, 
                 **keyword_args):
        if world is None:
            core.World.__init__(self,*positional_args, **keyword_args)
            self.update_geometric()
        else:
            raise NotImplemented
        self._drawer = WorldDrawer(world=self, scale=1.)

    def register(self, obj):
        core.World.register(self, obj)
        self._drawer.register(obj)

    def init_graphic(self, *positional_args, **keyword_args):
        self._viewer = init_viewer(
            self._drawer.root, *positional_args, **keyword_args)
        self._viewer.realize()

    def update_graphic(self):
        self._drawer.update()
        self._viewer.frame()

    def graphic_is_done(self):
        return self._viewer.done()


class DrawerPlugin(core.Plugin):

    def __init__(self, scale=1.,options=None):
        self._drawer = WorldDrawer(scale=scale, options=options)

    def init(self, world, time):
        world.update_geometric()
        self._drawer.init(world)
        self._viewer = init_viewer(self._drawer.root)
        self._viewer.realize()

    def update(self, t ,dt):
        self._drawer.update()
        self._viewer.frame()

