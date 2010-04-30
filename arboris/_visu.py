# coding=utf-8
from abc import ABCMeta, abstractmethod, abstractproperty
from numpy import eye
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
        body_palette = []
        ncolor = 20
        for k in range(ncolor):
            h = 360./ncolor * k
            body_palette.append(hsv_to_rgb((h, 0.9 , 0.9)))
        options = {
            'display frame arrows': True,
            'display links': True,
            'display names': False,
            'display inertia ellipsoids': False,
            'display shapes': True,
            'frame arrows length': 0.08 * scale,
            'frame arrows radius': 0.005 * scale,
            'frame arrows alpha': 1.,
            'link radius': 0.004 * scale,
            'point radius': 0.008 * scale,
            'text size': 0.1 * scale,
            'body palette': body_palette,
            'center of interest': (0., 0., 0.),
            'camera position': (3.,3.,3.),
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

    #@abstractmethod
    def create_link(self):
        pass

    #@abstractmethod
    def create_ellipsoid(self):
        pass

    #@abstractmethod
    def create_sphere(self, radius, color):
        pass

    @abstractmethod
    def create_box(self, half_extents, color):
        pass

    #@abstractmethod
    def create_point(self, color):
        pass

    #@abstractmethod
    def create_cylinder(self, length, radius, color):
        pass

    @abstractmethod
    def add_child(self, parent, child):
        pass

class AnimatorDriver(DrawerDriver):

    __metaclass__ = ABCMeta

    @abstractmethod
    def update_transform(self, transform, pose):
        pass

    @abstractmethod
    def do_frame(self):
        pass

class Drawer(object):
    """
    Draw a world

    it does not animate it !

    TODO : use colors
    TODO : add shapes
    TODO : add links
    """
    
    def __init__(self, driver, flat=False):
        self.transform_nodes = {}
        self.body_colors = {}
        self._flat = flat
        # TODO: guess driver
        assert isinstance(driver, DrawerDriver)
        self._driver = driver
    
    def init_parse(self, ground, up, current_time):
        self._ground_node = self._driver.add_ground(up)
        self.frame_nodes = {ground: self._ground_node}

    def _add_frame(self, pose, is_constant, name):
        node = self._driver.create_transform(pose, is_constant, name)
        self._driver.add_child(node, self._driver.create_frame_arrows())
        #TODO: add frame name as node
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
        if isinstance(f1, arboris.core.Body):
            # no need to add a body
            bd1 = f1
            if self._flat:
                f1_node = self._add_frame(bd1.pose, False, f1.name)
                d.add_child(self._ground_node, f1_node)
            else:
                f1_node = self._add_frame(j.pose, False, f1.name)
                d.add_child(self.frame_nodes[f0], f1_node)
            self.frame_nodes[f1] = f1_node
        elif isinstance(f1, arboris.core.MovingSubFrame):
            # this should never happen
            raise ValueError()
        elif isinstance(f1, arboris.core.SubFrame):
            bd1 = f1.body
            if self._flat:
                bd1_node = self._add_frame(bd1.pose, False, bd1.name)
                d.add_child(self.ground_node, bd1_node)
                f1_node = self._add_frame(f1.bpose, True, f1.name)
                d.add_child(bd1_node, f1_node)
            else:
                f1_node = self._add_frame(j.pose, False, f1.name)
                d.add_child(self.frame_nodes[f0], f1_node)
                bd1_node = self._add_frame(inv(f1.bpose), True, bd1.name)
                d.add_child(f1_node, bd1_node)
            self.frame_nodes[bd1] = bd1_node
            self.frame_nodes[f1] = f1_node
        if not self._flat:
            self.transform_nodes[j] = f1_node
    
    def register(self, obj):
        if isinstance(obj, arboris.core.SubFrame) and \
                not(obj in self.frame_nodes):
            node = self._add_frame(obj.bpose, True, obj.name)
            self.frame_nodes[obj] = node
            self._driver.add_child(self.frame_nodes[obj.body], node)
        if isinstance(obj, arboris.core.MovingSubFrame) and \
                not(obj in self.frame_nodes):
            node = self._add_frame(obj.bpose, False, obj.name)
            self.frame_nodes[obj] = node
            self.transform_nodes[obj] = node
            self._driver.add_child(self.frame_nodes[obj.body], node)
        if isinstance(obj, arboris.shapes.Box):
            #TODO: fix color
            node = self._driver.create_box(obj.half_extents, (1,0,0,1))
            self._driver.add_child(self.frame_nodes[obj.frame], node)
    def finish(self):
        self._driver.finish()

