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
import logging

logging.basicConfig(level=logging.DEBUG)

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


class SwitcherHandler(osgGA.GUIEventHandler):
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
        'plane half extents': (1. * scale, 1. * scale),
        'text size': 0.1 * scale,
        'body palette': body_palette,
        'fullscreen': False,
        'window size': (800, 600),
        'window position': (0, 0),
        'center of interest': (0., 0., 0.),
        'camera position': (3.,3.,3.),
        'force length': 0.1 * scale,
        'force radius': 0.002 * scale}
    return options


class Drawer(arboris.core.Observer):
    """Draw the world, creating OSG nodes.

    **Attributes:**
    - :attr:`root`: the root node of the OSG scene,
    - :attr:`frames`: a dictionnary of tranform nodes corresponding to
      and indexed by :class:`arboris.core.Frame` objects,
    - :attr:`constraint_forces` and :attr:`constraints_moment`: 
      dictionnaries of tranform nodes corresponding to and indexed by
      :class:`arboris.core.Constraint` objects,

    **Methods:**
    - :meth:`__init__` creates ``root`` OSG node, get the graphic options
      (colors, sizes...) and do house keeping. If the ``world`` parameter is
      given, it will be converted to OSG nodes (it calls :meth:`init`). 
    - :meth:`init` converts the ``world`` to osg nodes 
      (it calls :meth:`register`).
    - :meth:`register` converts a single arboris object ``obj`` to zero or more
      OSG nodes.
    
    """
    def __init__(self, world, scale=1., options=None, viewer=None):
        arboris.core.Observer.__init__(self)
        if options is None:
            self._options = graphic_options(scale)
        else:
            self._options = options
        self.root = osg.Group()
        # enable the transparency/alpha:
        blend = osg.StateSet()
        blend.setMode(osg.GL_BLEND, osg.StateAttribute.ON)
        self.root.setStateSet(blend)
        
        self._generic_frame = draw_frame(
            length=self._options['frame length'],
            radius=self._options['frame radius'],
            alpha=self._options['frame alpha'])
        self._generic_frame.setNodeMask(_MASK['frame'])
        self._generic_force = draw_force(
            length= self._options['force length'],
            radius = self._options['force radius'])
        self._body_colors = {}
        self.registered = []
        self.frames = {}
        self.constraint_forces = {}
        self._world = world
        self._register(self._world.ground)
        if viewer is None:
            # create the osg viewer:
            self.viewer = osgViewer.Viewer()
            self.viewer.setSceneData(self.root)
            manipulator = osgGA.TrackballManipulator()
            self.viewer.setCameraManipulator(manipulator)
            # set the position of the camera/view:
            coi = map(float, self._options['center of interest'])
            cam = map(float, self._options['camera position'])
            up = map(float, self._world.up)
            manipulator.setHomePosition(
                osg.Vec3d(coi[0]+cam[0], coi[1]+cam[1], coi[2]+cam[2]),
                osg.Vec3d(coi[0], coi[1], coi[2]),
                osg.Vec3d(up[0], up[1], up[2]))
            # setup the window:
            if self._options['fullscreen'] is False:
                window_size = self._options['window size']
                window_pos = self._options['window position']
                self.viewer.setUpViewInWindow(
                    window_pos[0], window_pos[1], 
                    window_size[0], window_size[1])
            self.viewer.home()
        else:
            self.viewer = viewer
        camera = self.viewer.getCamera()
            #inheritanceMask = (~(osg.CullSettings.CULL_MASK) & osg.CullSettings.ALL_VARIABLES)
            #camera.setInheritanceMask(inheritanceMask)
        # add an handler for GUI Events (will toggle display on/off):
        self.handler = SwitcherHandler(camera)
        self.viewer.addEventHandler(self.handler.__disown__())

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

    def _register(self, obj):
        """Add the given arboris object ``obj`` to the scene.

        For a single arboris object, several OSG nodes may be added,
        belonging to different categories. For instance,
        registering an object of the :class:`Body` class returns nodes
        belonging to the ``frame``, ``name`` and ``inertia ellispoid``
        categories.

        The categories a node belongs to is specified using node masks.

        """
        if obj in self.registered:
            pass
        else:
            self.registered.append(obj)
            opts = self._options
            if isinstance(obj, arboris.core.Frame):
                # create a transform for the frames (instances of 
                # the Body and Subframe classes)
                t = osg.MatrixTransform()
                self.frames[obj] = t
                # draw the frame name:
                name = draw_text(obj.name, opts['text size'])
                name.setNodeMask(_MASK['name'])
                t.addChild(name)
                # draw the frame basis:
                t.addChild(self._generic_frame)
                if isinstance(obj, arboris.core.Body):
                    self.root.addChild(t)
                    if obj.mass[5,5] != 0:
                        # draw the body inertia ellipsoid.
                        #
                        # MatrixTransform() # position
                        #  |
                        # PositionAttitudeTransform() # ellispoid axis scale
                        #  |
                        # Geode()
                        #  |
                        # Sphere()
                        #
                        Mb = obj.mass
                        color = self._choose_color(obj)
                        bHg = arboris.massmatrix.principalframe(Mb)
                        Mg = arboris.massmatrix.transport(Mb, bHg)
                        shape = osg.ShapeDrawable(
                            osg.Sphere(osg.Vec3(0.,0.,0.), 1.))
                        shape.setColor(color)
                        shape_geo = osg.Geode()
                        shape_geo.addDrawable(shape)
                        scale_node = osg.PositionAttitudeTransform()
                        scale_node.setScale(
                            osg.Vec3d(Mg[0,0], Mg[1,1], Mg[2,2]))
                        scale_node.addChild(shape_geo)
                        gen_scale_node = osg.PositionAttitudeTransform()
                        gen_scale_node.addChild(scale_node)
                        pos_node = osg.MatrixTransform()
                        pos_node.setMatrix(_pose2mat(bHg))
                        pos_node.addChild(gen_scale_node)
                        pos_node.setNodeMask(_MASK['inertia ellipsoid'])
                        self.frames[obj].addChild(pos_node)
                elif isinstance(obj, arboris.core.SubFrame) or\
                        isinstance(obj, arboris.core.MovingSubFrame):
                    self.frames[obj.body].addChild(t)
                    # draw a line between the subframe and its parent:
                    color = self._choose_color(obj.body)
                    nl = draw_line((0,0,0),
                               -dot(obj.bpose[0:3,0:3].T, obj.bpose[0:3,3]),
                               opts['link radius'], color)
                    nl.setNodeMask(_MASK['link'])
                    self.frames[obj]. addChild(nl)
            elif isinstance(obj, arboris.constraints.BallAndSocketConstraint)\
                    or isinstance(obj, arboris.constraints.SoftFingerContact):
                t = osg.PositionAttitudeTransform()
                self.constraint_forces[obj] = t
                t.setNodeMask(_MASK['constraint force'])
                self.frames[obj._frames[0]].addChild(t)
                t.addChild(self._generic_force)
            elif isinstance(obj, arboris.core.Shape):
                color = self._choose_color(obj.frame.body)
                if isinstance(obj, arboris.shapes.Plane):
                    # instead of drawing an infinite plane, we draw a finite
                    # square.
                    from arboris.collisions import _normal_to_frame
                    dx, dy = opts['plane half extents']
                    points = [(-dx, dy, 0),
                              (-dx, -dy, 0),
                              (dx, -dy, 0),
                              (dx, dy, 0)]
                    H = _normal_to_frame(obj.coeffs[0:3])
                    origin = obj.coeffs[3]*obj.coeffs[0:3]
                    vertices = osg.Vec3Array()
                    for point in points:
                        vertex = origin + dot(H[0:3, 0:3], point)
                        vertices.push_back(osg.Vec3(vertex[0],
                                                    vertex[1],
                                                    vertex[2]))
                    shape = osg.Geometry()
                    shape.setVertexArray(vertices)
                    face = osg.DrawElementsUInt(osg.PrimitiveSet.QUADS,0)
                    face.push_back(3)
                    face.push_back(2)
                    face.push_back(1)
                    face.push_back(0)
                    shape.addPrimitiveSet(face)
                    colors = osg.Vec4Array()
                    colors.push_back(osg.Vec4(color[0],
                                              color[1],
                                              color[2],
                                              color[3]))
                    shape.setColorArray(colors)
                    shape.setColorBinding(osg.Geometry.BIND_OVERALL)
                else:
                    if isinstance(obj, arboris.shapes.Sphere):
                        shape = osg.ShapeDrawable(
                            osg.Sphere(osg.Vec3(0.,0.,0.), obj.radius))
                    elif isinstance(obj, arboris.shapes.Point):
                        shape = osg.ShapeDrawable(
                            osg.Sphere(osg.Vec3(0.,0.,0.),
                                       self._options['point radius']))
                    elif isinstance(obj, arboris.shapes.Box):
                        shape = osg.ShapeDrawable(
                            osg.Box(osg.Vec3(0.,0.,0.), obj.half_extents[0]*2.,
                                    obj.half_extents[1]*2, obj.half_extents[2]*2))
                    elif isinstance(obj, arboris.shapes.Cylinder):
                        shape = osg.ShapeDrawable(
                            osg.Cylinder(osg.Vec3(0.,0.,0.),
                                         obj.radius, obj.length))
                    else:
                        raise NotImplementedError(
                                'Cannot draw "{0}"'.format(obj))
                    shape.setColor(color)
                switch = osg.Switch()
                geode =  osg.Geode()
                geode.addDrawable(shape)
                geode.setNodeMask(_MASK['shape'])
                self.frames[obj.frame].addChild(geode)
            elif isinstance(obj, arboris.core.Joint) or \
                isinstance(obj, arboris.core.Constraint) or \
                isinstance(obj, arboris.core.Controller):
                pass
            else:
                raise NotImplementedError('Cannot draw "{0}"'.format(obj))

    def init(self, world, timeline):
        self._world.update_geometric() #TODO find a way to remove this
        for obj in self._world.iterbodies():
            self._register(obj)
        for obj in self._world.itersubframes():
            self._register(obj)
        for obj in self._world.itershapes():
            self._register(obj)
        for obj in self._world._constraints:
            self._register(obj)
        for key, name, on in (
            (ord('f'), 'frame', True),
            (ord('i'), 'inertia ellipsoid', False),
            (ord('l'), 'link', True),
            (ord('n'), 'name', False),
            (ord('w'), 'constraint force', True),
            (ord('g'), 'shape', True)):
            self.handler.add_category(key, name, on)

    def update(self, dt=None):
        for obj in self._world.itersubframes():
            self.frames[obj].setMatrix(_pose2mat(obj.bpose))
        for obj in self._world.iterbodies():
            self.frames[obj].setMatrix(_pose2mat(obj.pose))
        for obj in self._world.iterconstraints():
            if obj.is_active():
                from numpy.linalg import norm
                # scale the generic_force according to the force
                # norm. We constraint the scaling to be in [0.1, 10]
                if isinstance(obj, arboris.constraints.SoftFingerContact):
                    force = obj._force[1:4]
                elif isinstance(obj,
                        arboris.constraints.BallAndSocketConstraint):
                    force = obj._force[0:3]
                scale = min(10., max(0.1, norm(force)/100))
                self.constraint_forces[obj].setScale(
                        osg.Vec3d(scale,scale,scale))
                _align_z_with_vector(force,
                                     self.constraint_forces[obj])
                self.constraint_forces[obj].setNodeMask(
                        _MASK['constraint force'])
            else:
                self.constraint_forces[obj].setNodeMask(0)
        self.viewer.frame()

    def done(self):
        return self.viewer.done()

    def finish(self):
        pass

