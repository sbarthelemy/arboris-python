# coding=utf-8
"""
Visualization of a simulation

This module is based on the openscenegraph (osg) python wrappers.

It can be used in four ways:

    - *offline*: during the simulation, graphics are saved by an 
      :class:`OsgSavePlugin` plugin and played later (offline), 
      (TODO: not available yet)
    
    - *during simulation*: graphics are drawn and updated from the main
      simulation loop by a the :class:`DrawerPlugin` plugin,
    
    - *interactive GUI*: TODO

    - *interactive CLI*: using :class:`DrawableWorld`.

Scene graph basics
------------------

A scene graph is a data structure that arranges the logical and 
spatial representation of a graphical scene. It consists of a 
collection of *nodes* in a tree or graph structure. In general a node
may have many children but only a single parent, with the effect of a 
parent apparent to all its child nodes. For instance, a geometrical 
transformation matrix node would move all its children.

In some cases, a single (child) node may be shared between several 
parents, for instance when the same geometry is displayed several times
simultaneously (this saves memory).

OSG in 20"
----------

TODO: finish

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
capabilities, so a single Viewer-based application executable runs on 
single or multiple display systems.

Internals
---------

The :class:`WorldDrawer` class creates a graphic representation of an
arboris world as an OSG graph. Several properties of the world are 
represented (shapes, inertia ellispoids,...), and the graphics can be 
tuned with a dict of options, whose default values are given by the 
:func:`graphic_option` function. The object stores a ref to the arboris
world in order to update the representation when the bodies move.

The :func:`init_viewer` function creates an OSG Viewer and an OSG
GUIEventHandler. It is the place where window size, camera position
and keyboard shortcuts are set.

The end user can draw a world through, at choice, a :class:`DrawerPlugin`
object, or superseding the "raw" :class:`core.World` object by a 
:class:`visuosg.DrawableWorld` one.

"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>",
              "Joseph SALINI <joseph.salini@gmail.com>")

import osg, osgDB, osgGA, osgViewer, osgText, osgUtil
from numpy import pi, arctan2, array, dot, cross, sqrt, eye, cos, sin
import shapes
import core
import homogeneousmatrix as Hg
import massmatrix
import logging

logging.basicConfig(level=logging.DEBUG)

def draw_frame(length=1., radius=0.05, alpha=1.):
    """Draw a frame, as 3 cylinders.

    :param length: the cylinders length
    :type length: float
    :param radius: the cylinders radius
    :type radius: float
    :param alpha: the cylinders alpha value
    :type alpha: float
    :rtype: osg.PositionAttitudeTransform

    **Example:**

    >>> generic_frame = draw_frame(.5, .8)
    
    """
    # create the x cylinder
    cyl_x = osg.ShapeDrawable(osg.Cylinder(
        osg.Vec3(0.,0.,length/2.), radius, length))
    cyl_y = osg.ShapeDrawable(cyl_x)
    cyl_z = osg.ShapeDrawable(cyl_x)
    cyl_x.setColor(osg.Vec4(1.,0.,0.,alpha))
    cyl_y.setColor(osg.Vec4(0.,1.,0.,alpha))
    cyl_z.setColor(osg.Vec4(0.,0.,1.,alpha))
    # create the geode for each cylinder
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
    :param radius: the cylinder radius
    :type radius: float
    :param color: the line color 
    :type color: osg.Vec4
    :rtype: osg.Transform

    **Example:**

    >>> draw_line(array((1.,2.,3.)), array((4.,5.,6.)), radius=.5) #doctest: +ELLIPSIS
    <osg.PositionAttitudeTransform; proxy of <Swig Object of type 'osg::PositionAttitudeTransform *' at 0x...> >

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
    :param size: a scaling factor
    :type size: float

    **Example:**

    >>> draw_text('le texte a afficher', .2) #doctest: +ELLIPSIS
    <osg.Geode; proxy of <Swig Object of type 'osg::Geode *' at 0x...> >
    """
    text = osgText.Text()
    text.setCharacterSize(size)
    text.setText(str(label))
    text.setDrawMode(osgText.Text.TEXT | osgText.Text.BOUNDINGBOX)
    text.setAlignment(osgText.Text.CENTER_TOP)
    #text.setAxisAlignment(osgText.Text.SCREEN) # un-comment for HUD version
    geode = osg.Geode()
    geode.addDrawable(text)
    return geode

def graphic_options(scale=1.):
    """Default dictionnary of graphic options.
    
    :param scale: the scaling factor
    :type scale: float
    :return: scaled graphic options
    :rtype: dict

    """
    body_palette = []
    ncolor = 20
    for k in range(ncolor):
        h = 360./ncolor * k
        body_palette.append(hsv_to_rgb((h, 0.9 , 0.9)))
    options = {
        'frame length': 0.08 * scale,
        'frame radius': 0.005 * scale,
        'frame alpha': 1.,
        'link radius': 0.004 * scale,
        'point radius': 0.008 * scale,
        'text size': 0.1 * scale,
        'body palette': body_palette}
    return options


class WorldDrawer(object):
    """Draw the world, creating OSG nodes.

    **Attributes:**
    - :attr:`root`: the root node of the OSG scene,
    - :attr:`transforms`: a dictionnary of tranform nodes corresponding to and
      indexed by :class:`core.Frame` objects,
    - :attr:`switches`: a dictionnary indexed by arboris objects, whose values
      are dictionnaries too. These latter dictionaries values are OSG switches
      (which switch on/off the display of their children), while their keys 
      are strings, explaining what is displayed (``name``, 
      ``inertia ellipsoid``...).

    **Methods:**
    - :meth:`__init__` creates ``root`` OSG node, get the graphic options
      (colors, sizes...) and do house keeping. If the ``world`` parameter is
      given, it we be converted to OSG nodes (it calls :meth:`init`). 
    - :meth:`init` converts the ``world`` to osg nodes 
      (it calls :meth:`register`).
    - :meth:`register` converts a single arboris object ``obj`` to zero or more
      OSG nodes. It populates :attr:`transforms`:
    
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
            'inertia ellipsoid': False,
            'link': True,
            'name': False,
            'shape': True}
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
        """Select a colour in the palette for the body.
        """
        if body in self._body_colors:
            color = self._body_colors[body]
        else:
            try:
                c = self._options['body palette'].pop()
                color = osg.Vec4(c[0], c[1], c[2], alpha)
                self._body_colors[body] = color
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

        - ``name`` for any instance of the :class:`arboris.core.NamedObject` class
        - ``frame`` for objects of the :class:`arboris.core.Frame` class
        - ``link`` lines for skeletton-like view
        - ``shapes`` the basic shapes (:class:`arboris.core.Shape`) 
          used in the simulation for contact detection
        - ``geometry``
        - ``cog``

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
                color = self._choose_color(obj)
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
                    
                    bHg = massmatrix.principalframe(Mb)
                    Mg = massmatrix.transport(Mb, bHg)
                    shape = osg.ShapeDrawable(
                        osg.Sphere(osg.Vec3(0.,0.,0.), 1.))
                    shape.setColor(color)
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
                color = self._choose_color(obj.frame.body)
                if isinstance(obj, shapes.Sphere):
                    shape = osg.ShapeDrawable(
                        osg.Sphere(osg.Vec3(0.,0.,0.), obj.radius))
                elif isinstance(obj, shapes.Point):
                    shape = osg.ShapeDrawable(
                        osg.Sphere(osg.Vec3(0.,0.,0.), 
                                   self._options['point radius']))
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
                geode =  osg.Geode()
                geode.addDrawable(shape)
                switches['shape'].addChild(geode)
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

            if isinstance(obj, core.NamedObject) and (obj.name is not None):
                switches['name'] = osg.Switch()
                switches['name'].addChild(
                    draw_text(obj.name, opts['text size']))
            if isinstance(obj, core.Frame):
                switches['frame'] = osg.Switch()
                switches['frame'].addChild(self._generic_frame)
            if (parent is not None) and (len(switches) != 0):
                self.switches[obj] = {}
                for key, val in switches.items():
                    parent.addChild(val)
                    self.switches[obj][key] = val
                    if self.is_displayed[key]:
                        val.setAllChildrenOn()
                    else:
                        val.setAllChildrenOff()

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
    
def init_viewer(drawer, fullscreen=False, window=(0,0,800,600), 
                coi=(0,0,0), camera=(3,3,3), up=(0,1,0)):
        # create the osg viewer:
        viewer = osgViewer.Viewer()
        viewer.setSceneData(drawer.root)
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
        kbh = SwitcherHandler(drawer)
        viewer.addEventHandler(kbh.__disown__())
        return viewer


class SwitcherHandler(osgGA.GUIEventHandler):
    """Switches parts of the display on/off according to keys pressed.
    """
    def __init__(self, drawer):
        osgGA.GUIEventHandler.__init__(self)
        self._drawer = drawer
        self._logger = logging.getLogger(self.__class__.__name__)

    def handle(self, ea, aa, obj, nv):

        def _update_action(action, drawer):
            if action is not None:
                if action == ord('f'):
                    drawer.switch('frame')
                elif action == ord('l'):
                    drawer.switch('link')
                elif action == ord('n'):
                    drawer.switch('name')
                elif action == ord('g'):
                    drawer.switch('shape')
                elif action == ord('i'):
                    drawer.switch('inertia ellipsoid')
                elif action == 65362: # up
                    drawer.move_frame([1,0,0,0,0,0])
                elif action == 65364: # down
                    drawer.move_frame([-1,0,0,0,0,0])
                elif action == 65361: # left
                    drawer.move_frame([0,0,0,0,0,-1])
                elif action == 65363: # right
                    drawer.move_frame([0,0,0,0,0,1])

        eventtype = ea.getEventType()
        if eventtype == ea.KEYDOWN:
            action = ea.getKey()
            if action < 256:
                self._logger.info('action %d: %s', action, chr(action))
            else:
                self._logger.info('action %d', action)
            _update_action(action, self._drawer)
        return False


class JointIkHandler(osgGA.GUIEventHandler):
    """Move the mouse-selected joint with keyboard.
    """

    def __init__(self, drawer, viewer, keymap='us'):
        osgGA.GUIEventHandler.__init__(self)
        self._drawer = drawer
        self._viewer = viewer
        self._frame = None
        self._logger = logging.getLogger(self.__class__.__name__)
        if keymap == 'us':
            self._key_to_twist = {
                ord('w'): (0,0,0,1,0,0),
                ord('s'): (0,0,0,-1,0,0),
                ord('d'): (0,0,0,0,0,1),
                ord('a'): (0,0,0,0,0,-1),
                ord('q'): (0,0,0,0,1,0),
                ord('e'): (0,0,0,0,-1,0),
                23: (1,0,0,0,0,0), # CTRL+w
                19: (-1,0,0,0,0,0), # CTRL+s
                4: (0,0,1,0,0,0), # CTRL+d
                1: (0,0,-1,0,0,0), # CTRL+a
                17: (0,1,0,0,0,0), # CTRL+q
                5: (0,-1,0,0,0,0), # CTRL+e
            }
        elif  keymap == 'fr':
            self._key_to_twist = {
                ord('z'): (0,0,0,1,0,0),
                ord('s'): (0,0,0,-1,0,0),
                ord('d'): (0,0,0,0,0,1),
                ord('q'): (0,0,0,0,0,-1),
                ord('a'): (0,0,0,0,1,0),
                ord('e'): (0,0,0,0,-1,0),
                26: (1,0,0,0,0,0), # CTRL+z
                19: (-1,0,0,0,0,0), # CTRL+s
                4: (0,0,1,0,0,0), # CTRL+d
                17: (0,0,-1,0,0,0), # CTRL+q
                1: (0,1,0,0,0,0), # CTRL+a
                5: (0,-1,0,0,0,0), # CTRL+e
            }

    def _move_frame(self, twist):
        #TODO: express twist in the basis of self._frame?
        assert isinstance(self._frame, core.Frame)
        from numpy.linalg import pinv
        joint = self._frame.body.parentjoint
        if joint is not None:
            J = joint.jacobian
            nu = dot(pinv(J),twist)
            self._logger.info(('nu',nu))
            joint.gvel[:] = nu
            dt = 0.01
            joint.integrate(dt)

    def handle(self, ea, aa, obj, nv):
        eventtype = ea.getEventType()
        if eventtype == ea.PUSH:
            inter = osgUtil.LineSegmentIntersector(
                osgUtil.LineSegmentIntersector.PROJECTION, 
                ea.getXnormalized(), 
                ea.getYnormalized())
            iv = osgUtil.IntersectionVisitor(inter)
            self._viewer.getCamera().accept(iv)
            if inter.containsIntersections():
                pick = inter.getFirstIntersection()
                result = osg.NodeToMatrixTransform(pick.nodePath[2])
                for k, v in self._drawer.transforms.iteritems():
                    if v.this == result.this:
                        assert isinstance(k,core.Frame)
                        self._frame = k
                        self._logger.info('selected frame: %s named "%s"', k,
                                          k.name)
                        break
        elif eventtype == ea.KEYDOWN:
            action = ea.getKey()
            if (self._frame is not None) and (action is not None):
                try:
                    twist = self._key_to_twist[action]
                    self._move_frame(twist)
                except KeyError:
                    pass
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
            self._drawer, *positional_args, **keyword_args)
        self._viewer.realize()

    def update_graphic(self):
        self._drawer.update()
        try:
            self._viewer.frame()
        except AttributeError:
            raise Error('You should call ``init_graphic()`` once before calling ``update_graphic``')

    def graphic_is_done(self):
        return self._viewer.done()


class DrawerPlugin(core.Plugin):

    def __init__(self, scale=1.,options=None):
        self._drawer = WorldDrawer(scale=scale, options=options)

    def init(self, world, time):
        world.update_geometric()
        self._drawer.init(world)
        self._viewer = init_viewer(self._drawer)
        self._viewer.realize()

    def update(self, t, dt):
        self._drawer.update()
        self._viewer.frame()


def hsv_to_rgb(hsv):
    """Convert color from hsv to rgb.

    :param hsv: hsv values (with h in [0..360], s and v in [0..1])
    :type hsv: 3-tuple of floats (h, s, v)
    :return rgb: rgb values (with r, g and b in [0..1])
    :rtype rgb: 3-tuple of floats (r, g, b)

    The algorithm is taken from:
    http://en.wikipedia.org/wiki/HSL_and_HSV

    **Examples:**

    >>> hsv_to_rgb((360,1.,1.)) # red
    (1.0, 0.0, 0.0)
    >>> hsv_to_rgb((360,0.5,1.)) # faded red
    (1.0, 0.5, 0.5)
    >>> hsv_to_rgb((360,1.,0.5)) # dark red
    (0.5, 0.0, 0.0)
    >>> hsv_to_rgb((120,1.,1)) # green
    (0.0, 1, 0.0)
    >>> hsv_to_rgb((240,1.,1)) # blue
    (0.0, 0.0, 1)

    """
    from math import floor
    (h, s, v) = hsv
    hi = floor(h/60) % 6
    f = h/60. - floor(h/60)
    p = v * (1 - s)
    q = v * (1 - f*s)
    t = v * (1 - (1-f)*s)
    if hi == 0:
        rgb = (v, t, p)
    elif hi == 1:
        rgb = (q, v, p)
    elif hi == 2:
        rgb = (p, v, t)
    elif hi == 3:
        rgb = (p, q, v)
    elif hi == 4:
        rgb = (t, p, v)
    elif hi == 5:
        rgb = (v, p, q)
    return rgb




