# coding=utf-8
from abc import ABCMeta, abstractmethod, abstractproperty
from numpy import eye
from numpy.linalg import norm
import arboris.core

def hsv_to_rgb(hsv):
    """Convert color from hsv to rgb.

    :param hsv: hsv values (with h in [0..360], s and v in [0..1])
    :type hsv: 3-tuple of floats (h, s, v)
    :return rgb: rgb values (with r, g and b in [0..1])
    :rtype rgb: 3-tuple of floats (r, g, b)

    The algorithm is taken from:
    http://en.wikipedia.org/wiki/HSL_and_HSV

    **Examples:**

    >>> hsv_to_rgb((360, 1., 1.)) # red
    (1.0, 0.0, 0.0)
    >>> hsv_to_rgb((360, 0.5, 1.)) # faded red
    (1.0, 0.5, 0.5)
    >>> hsv_to_rgb((360, 1., 0.5)) # dark red
    (0.5, 0.0, 0.0)
    >>> hsv_to_rgb((120, 1., 1)) # green
    (0.0, 1, 0.0)
    >>> hsv_to_rgb((240, 1., 1)) # blue
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


class DrawerDriver():
    """ABC for drawer drivers.

    This class defines an interface for "drawer drivers"
    which take care of the low level details of drawing the
    world, on behalf of the :class:`arboris.visu.Drawer`
    class.

    """

    __metaclass__ = ABCMeta

    def __init__(self, scale=1., options=None):
        if options is None:
            self._options = self.get_default_options(scale)
        else:
            self._options = options

    @staticmethod
    def get_default_options(scale=1.):
        """Get graphic options as a dict.

        :param scale: the scaling factor
        :type scale: float
        :return: scaled graphic options
        :rtype: dict

        """
        options = {
            'display frame arrows': True,
            'display links': True,
            #'display names': False,
            #'display inertia ellipsoids': False,
            'display shapes': True,
            'frame arrows length': 0.08 * scale,
            #'frame arrows radius': 0.005 * scale,
            #'frame arrows alpha': 1.,
            #'link radius': 0.004 * scale,
            'point radius': 0.008 * scale,
            'plane half extents': (1. * scale, 1. * scale),
            #'text size': 0.1 * scale,
            #'center of interest': (0., 0., 0.),
            #'camera position': (3.,3.,3.),
            'force length': 0.1 * scale,
            'force radius': 0.002 * scale}
        return options

    @abstractmethod
    def add_ground(self, up):
        pass

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def finish(self):
        pass

    @abstractmethod
    def create_transform(self, pose, is_constant, name=None):
        pass

    @abstractmethod
    def create_frame_arrows(self):
        pass

    @abstractmethod
    def create_line(self, start, end, color):
        pass

    @abstractmethod
    def create_ellipsoid(self, radii, color):
        pass

    def create_sphere(self, radius, color):
        return self.create_ellipsoid([radius] * 3, color)

    def create_point(self, color):
        return self.create_sphere(self._options['point radius'], color)

    @abstractmethod
    def create_box(self, half_extents, color):
        pass

    @abstractmethod
    def create_plane(self, coeffs, color):
        pass

    @abstractmethod
    def create_cylinder(self, length, radius, color):
        pass

    @abstractmethod
    def add_child(self, parent, child, category=None):
        pass

class AnimatorDriver(DrawerDriver):

    __metaclass__ = ABCMeta

    @abstractmethod
    def update_transform(self, transform, pose):
        pass

    @abstractmethod
    def do_frame(self):
        pass

class ColorGenerator(object):
    """Choose the color associated to an object.

    The color is represented by a 3-tupple of R, G, B values scaled between
    0 and 1. So

    - (1., 0., 0.) is red,
    - (1., 1., 1.) is white and
    - (0., 0., 0.) is black.

    The ``ColorGenerator`` manages a mapping whose keys are objects
    or object names and values are the associated colors.
    The color associated to an object can be requested through the
    :meth:`get_color` method.
    If neither the object nor its name are in the mapping, the color of its
    parent body is searched.

    If the parent body doesn't have color yet, the :meth:`generate_color`
    method is called, and the new color is added to the mapping.

    one can customize the color of an object using the :meth:`set_color` method
    or by providing a dict at the ``ColorGenerator`` creation.

    """

    def __init__(self, map=None, nb_colors=20):
        # init the color palette
        self._palette = []
        for k in range(nb_colors):
            h = 360./nb_colors * k
            self._palette.append(hsv_to_rgb((h, 0.9 , 0.9)))
        # init the mapping betwen objects and colors
        self._map = {}
        if map:
            for k, v in map.iteritems():
                self.set_color(k, v)

    def set_color(self, key, color):
        assert len(color) == 3 and all([v >= 0. and v <= 1. for v in color])
        self._map[key] = tuple(color)

    def get_color(self, obj):
        """Return the color mapped to the body or choose a new one.

        :param obj: the obj whose color is looked for
        :type obj: :class:`arboris.core.Frame` or :class:`arboris.core.Shape`
        :return: color as a 3-tuple of RGB values

        """
        def get_parent_body(obj):
            """Return the parent body of `obj`

            if `obj` is an instance of :class:`arboris.core.Body`,
            return itself.

            """
            if isinstance(obj, arboris.core.Frame):
                return obj.body
            elif isinstance(obj, arboris.core.Shape):
                return obj.frame.body
            else:
                raise ValueError()

        try:
            color = self._map[obj]
        except KeyError:
            try:
                color = self._map[obj.name]
            except KeyError:
                if isinstance(obj, arboris.core.Body):
                    color = self.generate_color(obj)
                    self.set_color(obj, color)
                else:
                    color = self.get_color(get_parent_body(obj))
        return color

    def generate_color(self, obj):
        try:
            return self._palette.pop()
        except IndexError:
            # the palette is exhausted
            return (1., 1., 1.)

class Drawer(object):
    """
    Draw a world

    it does not animate it !

    """

    def __init__(self, driver, flat=False, color_generator=None):
        self.transform_nodes = {}
        self._flat = flat
        # TODO: guess driver
        assert isinstance(driver, DrawerDriver)
        self._driver = driver
        if not color_generator:
            self._color_generator = ColorGenerator()
        else:
            self._color_generator = color_generator

    def init_parse(self, ground, up, current_time):
        self._ground_node = self._driver.add_ground(up)
        self.frame_nodes = {ground: self._ground_node}

    def _add_frame(self, parent, pose, is_constant, name, color):
        node = self._driver.create_transform(pose, is_constant, name)
        self._driver.add_child(parent, node)
        self._driver.add_child(node, self._driver.create_frame_arrows(),
                'frame arrows')
        #TODO: add frame name as node
        if is_constant and norm(pose[0:3, 3]) > 0:
            line = self._driver.create_line((0., 0., 0.), pose[0:3, 3], color)
            self._driver.add_child(parent, line, 'link')
        return node

    def add_link(self, f0, j, f1):
        """
        ::

        in arboris::

          f0------>f1
            j.pose

             f1.pose
             -------------------->
          gnd-------->bd1-------->f1
             bd1.pose    f1.bpose


        if not flat and bd1 is a SubFrame instance::

          f0n------>f1n------------->bd1n
             j.pose    inv(f1.bpose)

        if not flat and f1 is a Body instance::

          f0n------>f1n=bd1n
             j.pose

        if flat and f1 is a SubFrame instance::

          gndn-------->bd1n-------->f1n
              bd1.pose     f1.bpose

        if flat and f1 is a Body instance::

          gndn-------->f1n=bd1n
              bd1.pose

        """
        assert isinstance(f0, arboris.core.Frame)
        assert isinstance(j, arboris.core.Joint)
        assert isinstance(f1, arboris.core.Frame)
        d = self._driver
        bd1 = f1.body
        if isinstance(f1, arboris.core.Body):
            # no need to add a body
            if self._flat:
                f1_node = self._add_frame(self._ground_node,
                        bd1.pose, False, f1.name,
                        self._color_generator.get_color(f1))
            else:
                f1_node = self._add_frame(self.frame_nodes[f0],
                        j.pose, False, f1.name,
                        self._color_generator.get_color(f1))
            self.frame_nodes[f1] = f1_node
        elif isinstance(f1, arboris.core.MovingSubFrame):
            # this should never happen
            raise ValueError()
        elif isinstance(f1, arboris.core.SubFrame):
            if self._flat:
                bd1_node = self._add_frame(self.ground_node,
                        bd1.pose, False, bd1.name,
                        self._color_generator.get_color(bd1))
                f1_node = self._add_frame(bd1_node,
                        f1.bpose, True, f1.name,
                        self._color_generator.get_color(f1))
            else:
                f1_node = self._add_frame(self.frame_nodes[f0],
                        j.pose, False, f1.name,
                        self._color_generator.get_color(f1))
                bd1_node = self._add_frame(f1_node,
                        inv(f1.bpose), True, bd1.name,
                        self._color_generator.get_color(bd1))
            self.frame_nodes[bd1] = bd1_node
            self.frame_nodes[f1] = f1_node
        if not self._flat:
            self.transform_nodes[j] = f1_node

    def register(self, obj):
        if isinstance(obj, arboris.core.SubFrame) and \
                not(obj in self.frame_nodes):
            color = self._color_generator.get_color(obj)
            node = self._add_frame(self.frame_nodes[obj.body],
                    obj.bpose, True, obj.name, color)
            self.frame_nodes[obj] = node
        if isinstance(obj, arboris.core.MovingSubFrame) and \
                not(obj in self.frame_nodes):
            color = self._color_generator.get_color(obj)
            node = self._add_frame(self.frame_nodes[obj.body],
                    obj.bpose, False, obj.name, color)
            self.frame_nodes[obj] = node
            self.transform_nodes[obj] = node
        if isinstance(obj, arboris.core.Shape):
            color = self._color_generator.get_color(obj)
            if isinstance(obj, arboris.shapes.Sphere):
                node = self._driver.create_sphere(obj.radius, color)
            if isinstance(obj, arboris.shapes.Point):
                node = self._driver.create_point(color)
            if isinstance(obj, arboris.shapes.Cylinder):
                node = self._driver.create_cylinder(obj.length, obj.radius,
                                                    color)
            if isinstance(obj, arboris.shapes.Plane):
                node = self._driver.create_plane(obj.coeffs, color)
            if isinstance(obj, arboris.shapes.Box):
                node = self._driver.create_box(obj.half_extents, color)
            self._driver.add_child(self.frame_nodes[obj.frame], node, 'shape')

    def finish(self):
        self._driver.finish()
