# coding=utf-8
"""
Visualization of a simulation

This module is based on the openscenegraph (osg) python wrappers.

Scene graph basics
------------------

A scene graph is a data structure that arranges the logical and 
spatial representation of a graphical scene. It consists of a 
collection of *nodes* in a tree or graph structure. In general a node
may have many children but only a single parent, with the effect of a 
parent apparent to all its child nodes. For instance, a geometrical 
transformation matrix node would move all its children.

In some cases, a single (child) node may be shared between several 
parents, for instance when the same geometry is displayed at several places
(this saves memory).

"""
__author__ = (u"Sébastien BARTHÉLEMY <barthelemy@crans.org>",
              "Joseph SALINI <joseph.salini@gmail.com>")

import osg, osgDB, osgGA, osgViewer, osgText, osgUtil
from numpy import pi, arctan2, array, dot, cross, sqrt, eye, cos, sin
import arboris.shapes
import arboris.core
import arboris.constraints
import arboris.homogeneousmatrix as Hg
import arboris.massmatrix
from arboris._visu import hsv_to_rgb
import logging
import arboris._visu

logging.basicConfig(level=logging.DEBUG)

_MASK = {
        'name': 1<<1,
        'frame': 1<<2,
        'link': 1<<3,
        'shape': 1<<4,
        'inertia ellipsoid': 1<<5,
        'constraint force': 1<<6,
        'active constraint': 1<<7}

def _pose2mat(pose):
    """Convert an homogeneous transform matrix from numpy to osg.
    
    The conversion handles the transposition required by osg.

    :param pose: the homogeneous transform matrix.
    :type pose: (4,4)-shaped ndarray

    **Example:**

    >>> mat = array([[ 1.,  2.,  3.,  4.],
    ...              [ 5.,  6.,  7.,  8.],
    ...              [ 9., 10., 11., 12.],
    ...              [13., 14., 15., 16.]])
    >>> osg_mat = _pose2mat(mat)
    
    """
    m = osg.Matrixd()
    m.set(pose[0,0], pose[1,0], pose[2,0], pose[3,0],
          pose[0,1], pose[1,1], pose[2,1], pose[3,1],
          pose[0,2], pose[1,2], pose[2,2], pose[3,2],
          pose[0,3], pose[1,3], pose[2,3], pose[3,3],
          )
    return m

def _tuple2vec4(var):
    """Convert an iterable (of length 4) to an :class:``osg.Vec4``.

    :type var: tuple or any iterable of length 4
    :rtype: osg.Vec4

    """
    assert len(var) is 4
    return osg.Vec4(var[0], var[1], var[2], var[3])

def _align_z_with_vector(vec, transform):
    assert isinstance(transform, osg.PositionAttitudeTransform)
    z = array([0.,0.,1.])
    length = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
    if length > 0.:
        vec = vec/length
        q = cross(z, vec)
        sin_theta = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2])
        cos_theta = dot(z, vec)
        theta = arctan2(sin_theta, cos_theta)
        if sin_theta == 0.:
            transform.setAttitude(osg.Quat(theta, osg.Vec3d(1.,0.,0.)))
        else:
            transform.setAttitude(
                osg.Quat(theta, osg.Vec3d(q[0], q[1], q[2])))

class OsgDriver(arboris._visu.AnimatorDriver):

    def __init__(self, scale=1., options=None):
        if options is None:
            self._options = self.get_default_options(scale)
        else:
            self._options = options

    def init(self):
        pass

    @staticmethod
    def get_default_options(scale=1.):
        options = arboris._visu.DrawerDriver.get_default_options(scale)
        options['fullscreen'] = False
        options['window size'] = (800,600)
        options['window position'] = (0,0)
        return options

    def add_ground(self, up):
        # TODO: account for up
        # library of shared geometries
        self._generic_frame = _draw_frame(
            length=self._options['frame arrows length'],
            radius=self._options['frame arrows radius'],
            alpha=self._options['frame arrows alpha'])
        self._generic_frame.setNodeMask(_MASK['frame'])
        # TODO: add force etc
        self._root = osg.Group()
        # enable the transparency/alpha:
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self._root.setStateSet(blend)
        # create the osg viewer:
        self._viewer = osgViewer.Viewer()
        self._viewer.setSceneData(self._root)
        manipulator = osgGA.TrackballManipulator()
        self._viewer.setCameraManipulator(manipulator)
        # set the position of the camera/view:
        coi = map(float, self._options['center of interest'])
        cam = map(float, self._options['camera position'])
        up = map(float, up)
        manipulator.setHomePosition(
            osg.Vec3d(coi[0]+cam[0], coi[1]+cam[1], coi[2]+cam[2]),
            osg.Vec3d(coi[0], coi[1], coi[2]),
            osg.Vec3d(up[0], up[1], up[2]))
        # setup the window:
        if self._options['fullscreen'] is False:
            window_size = self._options['window size']
            window_pos = self._options['window position']
            self._viewer.setUpViewInWindow(
                    window_pos[0], window_pos[1],
                    window_size[0], window_size[1])
        self._viewer.home()
        camera = self._viewer.getCamera()
        #inheritanceMask = (~(osg.CullSettings.CULL_MASK) & osg.CullSettings.ALL_VARIABLES)
        #camera.setInheritanceMask(inheritanceMask)
        # add an handler for GUI Events (will toggle display on/off):
        self._handler = _SwitcherHandler(camera)
        for key, name, on in (
            (ord('f'), 'frame', True),
            (ord('i'), 'inertia ellipsoid', False),
            (ord('l'), 'link', True),
            (ord('n'), 'name', False),
            (ord('w'), 'constraint force', True),
            (ord('g'), 'shape', True)):
            self._handler.add_category(key, name, on)
        self._viewer.addEventHandler(self._handler.__disown__())
        return self._root

    def finish(self):
        pass
 
    def create_transform(self, pose, is_constant, name=None):
        t = osg.MatrixTransform()
        t.setMatrix(_pose2mat(pose))
        #TODO set name
        return t

    def create_frame_arrows(self):
        return self._generic_frame

    def create_link(self):
        pass

    def create_ellipsoid(self):
        pass

    def create_sphere(self, radius, color):
        shape = osg.ShapeDrawable(osg.Sphere(osg.Vec3(0.,0.,0.), radius))
        shape.setColor(_tuple2vec4(color))
        geode = osg.Geode()
        geode.addDrawable(shape)
        geode.setNodeMask(_MASK['shape'])
        return geode

    def create_box(self, half_extents, color):
        shape = osg.ShapeDrawable(
                osg.Box(osg.Vec3(0.,0.,0.), half_extents[0]*2.,
                    half_extents[1]*2, half_extents[2]*2))
        shape.setColor(_tuple2vec4(color))
        geode = osg.Geode()
        geode.addDrawable(shape)
        geode.setNodeMask(_MASK['shape'])
        return geode

    def create_point(self, color):
        shape = osg.ShapeDrawable(
                osg.Sphere(osg.Vec3(0.,0.,0.),
                self._options['point radius']))
        shape.setColor(_tuple2vec4(color))
        geode =  osg.Geode()
        geode.addDrawable(shape)
        geode.setNodeMask(_MASK['shape'])
        return geode

    def create_cylinder(self, length, radius, color):
        shape = osg.ShapeDrawable(
                osg.Cylinder(osg.Vec3(0.,0.,0.), radius, length))
        shape.setColor(_tuple2vec4(color))
        geode =  osg.Geode()
        geode.addDrawable(shape)
        geode.setNodeMask(_MASK['shape'])
        return geode

    def add_child(self, parent, child):
        parent.addChild(child)

    def update_transform(self, transform, pose):
        transform.setMatrix(_pose2mat(pose))

    def do_frame(self):
        self._viewer.frame()

def _draw_frame(length=1., radius=0.05, alpha=1.):
    """Draw a frame, as 3 cylinders.

    :param length: the cylinders length
    :type length: float
    :param radius: the cylinders radius
    :type radius: float
    :param alpha: the cylinders alpha value
    :type alpha: float
    :rtype: osg.PositionAttitudeTransform

    **Example:**

    >>> generic_frame = _draw_frame(.5, .8)
    
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
    geo_x.addDrawable(cyl_x)
    geo_y = osg.Geode()
    geo_y.addDrawable(cyl_y)
    geo_z = osg.Geode()
    geo_z.addDrawable(cyl_z)
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

    >>> draw_line((1.,2.,3.), (4.,5.,6.), radius=.5) #doctest: +ELLIPSIS
    <osg.PositionAttitudeTransform; proxy of <Swig Object of type 'osg::PositionAttitudeTransform *' at 0x...> >

    """
    v = array((end[0] - start[0], end[1] - start[1], end[2] - start[2]))
    length = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    v = v/length
    # create the cylinder
    cyl = osg.ShapeDrawable(
        osg.Cylinder(osg.Vec3(0., 0., length/2), radius, length))
    if color is not None:
        cyl.setColor(color)
    geode = osg.Geode()
    geode.addDrawable(cyl)
    line = osg.PositionAttitudeTransform()
    _align_z_with_vector(v, line)
    line.addChild(geode)
    return line

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
    geode = osg.Geode()
    geode.addDrawable(text)
    return geode

def draw_force(length=1., radius=0.03, alpha=1.):
    """Draw a force as a cylinder.

    :param length: the cylinder length
    :type length: float
    :param radius: the cylinder radius
    :type radius: float
    :param alpha: the cylinders alpha value
    :type alpha: float
    :rtype: osg.Geode
    
    """
    cyl = osg.ShapeDrawable(osg.Cylinder(
        osg.Vec3(0.,0.,length/2.), radius, length))
    cyl.setColor(osg.Vec4(1.,0.,0.,alpha))
    geode = osg.Geode()
    geode.addDrawable(cyl)
    return geode


class _SwitcherHandler(osgGA.GUIEventHandler):
    """Switches parts of the display on/off according to keys pressed.
    """
    def __init__(self, camera):
        osgGA.GUIEventHandler.__init__(self)
        self._logger = logging.getLogger(self.__class__.__name__)
        self._categories = {}
        self._camera = camera

    def add_category(self, key, category, on=True):
        self._categories[key] = category
        self._turn(category, on)

    def _toggle(self, category):
        """Toggle display of a category."""
        mask = self._camera.getCullMask()
        mask = mask ^ _MASK[category]
        self._camera.setCullMask(mask)
 
    def _turn(self, category, on):
        """Turn the display of a category on/off."""
        mask = self._camera.getCullMask()
        if on:
            mask = mask | _MASK[category]
        else:
            mask = mask & ~_MASK[category]
        self._camera.setCullMask(mask)

    def handle(self, ea, aa, obj, nv):
        eventtype = ea.getEventType()
        if eventtype == ea.KEYDOWN:
            key = ea.getKey()
            self._logger.info('key %d pressed', key)
            try:
                category = self._categories[key]
            except KeyError:
                return False
            mask = self._camera.getCullMask()
            mask = mask ^ _MASK[category]
            self._camera.setCullMask(mask)
        return False

class OsgObserver(arboris.core.Observer):

    def __init__(self):
        self._drawer = arboris._visu.Drawer(OsgDriver(), flat=False)

    def init(self, world, timeline):
        world.parse(self._drawer)

    def update(self, dt):
        for (t, n) in self._drawer.transform_nodes.iteritems():
            if isinstance(t, arboris.core.Joint):
                self._drawer._driver.update_transform(n, t.pose)
            elif isinstance(t, arboris.core.MovingSubFrame):
                self._drawer._driver.update_transform(n, t.bpose)
            else:
                raise ValueError()
        self._drawer._driver.do_frame()

    def finish(self):
        self._drawer.finish()

