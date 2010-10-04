# coding=utf-8

from xml.etree.ElementTree import ElementTree, SubElement, Element, Comment
from numpy import eye, zeros, all, array, ndarray, linspace, pi, linalg
from arboris.homogeneousmatrix import inv, rotzyx_angles, zaligned
import arboris.core
import arboris._visu
import subprocess
import os
import tempfile

def _indent(elem, level=0):
    """Indent the xml subtree starting at elem."""
    istr = "\t"
    i = "\n" + level*istr
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + istr
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            _indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

class ColladaDriver(arboris._visu.DrawerDriver):

    shapes = os.path.join(os.path.dirname(os.path.abspath(__file__)),
            'shapes.dae')

    def __init__(self, filename, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale, options)
        self._file = open(filename, 'w')
        self._file.write('<?xml version="1.0" encoding="utf-8"?>\n')
        self._colors = []

    def init(self):
        pass

    def add_ground(self, up):
        self._init_scene(up)
        return self.visual_scene

    def _init_scene(self, up):

        def asset(up):
            """Generate the "asset" collada tag."""
            asset = Element("asset")
            SubElement(asset, "created").text = '2005-06-27T21:00:00Z'  #TODO add date
            SubElement(asset, "modified").text = '2005-06-27T21:00:00Z' #TODO add date
            SubElement(asset, "unit", {"meter":"1", "name":"meter"}) #TODO useful ?
            if (up == [1., 0., 0.]).all():
                up = 'X_UP'
            elif (up == [0., 1., 0.]).all():
                up = 'Y_UP'
            elif (up == [0., 0., 1.]).all():
                up = 'Z_UP'
            else:
                up = None
                warning('the up vector is not compatible with collada')
            if up:
                SubElement(asset, "up_axis").text = up
            return asset


        def _create_scaled_frame_arrows(scale):
            node = Element("node", {'id': 'frame_arrows'})
            elem = SubElement(node, 'scale')
            elem.text = "{0} {0} {0}".format(scale)
            SubElement(node, "instance_node", {"url": self.shapes+"#frame_arrows"})
            return node

        self.collada = Element("COLLADA", {"version":"1.4.1",
                "xmlns":"http://www.collada.org/2005/11/COLLADASchema"})
        self.collada.append(asset(up))
        self.lib_materials = SubElement(self.collada, "library_materials")
        self.lib_effects = SubElement(self.collada, "library_effects")
        if self._options['display frame arrows']:
            lib_nodes = SubElement(self.collada, "library_nodes")
            lib_nodes.append(_create_scaled_frame_arrows(self._options['frame arrows length']))
        library_visual_scenes = SubElement(self.collada, "library_visual_scenes")
        scene_name = 'myscene'
        self.visual_scene = SubElement(library_visual_scenes, "visual_scene",
                {'id':scene_name})
        scene = SubElement(self.collada, "scene")
        SubElement(scene, "instance_visual_scene", {"url":'#'+scene_name})
        return self.visual_scene

    def add_child(self, parent, child, category=None):
        assert category in (None, 'frame arrows', 'shape', 'link')
        def plural(noun):
            """Return the plural of a noun"""
            if noun[-1] is 's':
                return noun
            else:
                return noun + 's'
        if category is None or self._options['display '+plural(category)]:
                parent.append(child)

    def create_transform(self, pose, is_constant, name=None):
        """Generate the node corresponding to the pose."""
        if name:
            node = Element("node", {"id":name, "name":name})
        else:
            node = Element("node")
        if not is_constant:
            rotz, roty, rotx = rotzyx_angles(pose)
            coeff = 180./pi
            matrix = SubElement(node, "matrix", {'sid':"matrix"})
            matrix.text = str(pose.reshape(16))[1:-1]
        elif not (pose == eye(4)).all():
            matrix = SubElement(node, "matrix", {'sid':'matrix'})
            matrix.text = str(pose.reshape(-1)).strip('[]')
        return node

    def create_line(self, start, end, color):
        vector = (array(end) - array(start))
        vector_norm = linalg.norm(vector)
        assert vector_norm > 0.
        n_vector = vector/vector_norm
        H = zaligned(n_vector)
        H[0:3, 3] = start
        node = self.create_transform(H, is_constant=True)
        scale = SubElement(node, 'scale')
        scale.text = "0. 0. {0}".format(vector_norm)
        elem = SubElement(node, "instance_geometry",
            {"url": self.shapes+"#line"})
        self._add_color(elem, color)
        return node

    def create_frame_arrows(self):
        return Element("instance_node", {"url": "#frame_arrows"})

    def create_box(self, half_extents, color):
        # instead of creating a new box, we use the #box and scale it
        # to the proper size
        node = Element("node")
        scale = SubElement(node, 'scale')
        scale.text = "{0} {1} {2}".format(*half_extents)
        elem = SubElement(node, "instance_geometry",
                          {"url": self.shapes+"#box"})
        self._add_color(elem, color)
        return node

    def create_plane(self, coeffs, color):
        H = zaligned(coeffs[0:3])
        H[0:3, 3] = coeffs[3] * coeffs[0:3]
        node = self.create_transform(H, is_constant=True)
        scale = SubElement(node, 'scale')
        scale.text = "{0} {1} 0.".format(*self._options["plane half extents"])
        elem = SubElement(node, "instance_geometry",
                {"url": self.shapes+"#plane"})
        self._add_color(elem, color)
        return node

    def _create_ellipsoid(self, radii, color, resolution):
        assert resolution in ('20', '80', '320')
        node = Element("node")
        scale = SubElement(node, 'scale')
        scale.text = "{0} {1} {2}".format(*radii)
        elem = SubElement(node, "instance_geometry",
                          {"url": self.shapes+"#sphere_"+resolution})
        self._add_color(elem, color)
        return node

    def create_ellipsoid(self, radii, color):
         return self._create_ellipsoid(radii, color, resolution='320')

    def create_point(self, color):
        radii = (self._options['point radius'],) * 3
        return self._create_ellipsoid(radii, color, resolution='20')

    def _create_cylinder(self, length, radius, color, resolution):
        assert resolution in ('8', '32')
        node = Element("node")
        scale = SubElement(node, 'scale')
        scale.text = "{0} {0} {1}".format(radius, length)
        elem = SubElement(node, "instance_geometry",
                          {"url": self.shapes+"#cylinder_"+resolution})
        self._add_color(elem, color)
        return node

    def create_cylinder(self, length, radius, color):
        return self._create_cylinder(length, radius, color, '32')


    def _add_color(self, parent, color):
        if not color in self._colors:
            self._colors.append(color)
        idx = self._colors.index(color)
        elem = SubElement(parent, "bind_material")
        elem = SubElement(elem, "technique_common")
        SubElement(elem, "instance_material",
                   {"symbol":"material",
                    "target": "#color"+str(idx)})

    def _write_colors(self):
        if len(self._colors):
            for c in self._colors:
                url = "color{0}".format(self._colors.index(c))
                se = SubElement(self.lib_materials, "material", {"id": url})
                SubElement(se, "instance_effect", {"url": "#"+ url+"_fx"})
                se = SubElement(self.lib_effects, "effect", {"id": url+"_fx"})
                se = SubElement(se, "profile_COMMON")
                se = SubElement(se, "technique", {"sid": "blender"})
                se = SubElement(se, "phong")
                se = SubElement(se, "diffuse")
                se = SubElement(se, "color")
                se.text = "{0} {1} {2} 1.".format(*c)
        else:
            self.collada.remove(self.lib_materials)
            self.collada.remove(self.lib_effects)

    def finish(self):
        # write to  file
        self._write_colors()
        _indent(self.collada)
        tree = ElementTree(self.collada)
        tree.write(self._file, "utf-8")
        self._file.close()

def write_collada_scene(world, dae_filename, flat=False):
    """Write a visual description of the scene in a collada file.

    :param world: the world to convert
    :type world: :class:`arboris.core.World` instance
    :param dae_filename: path of the output collada scene file
    :type dae_filename: str
    :param flat: if True, each body is a child of the ground. Otherwise
                 the hierarchy is copied from arboris
    :type flat: bool

    """
    assert isinstance(world, arboris.core.World)
    drawer = arboris._visu.Drawer(ColladaDriver(dae_filename), flat)
    world.parse(drawer)
    drawer.finish()

def write_collada_animation(collada_animation, collada_scene, hdf5_file,
                            hdf5_group="/"):
    """Combine a collada scene and an HDF5 file into a collada animation.

    :param collada_animation: path of the output collada animation file
    :type collada_animation: str
    :param collada_scene: path of the input collada scene file
    :type collada_scene: str
    :param hdf5_file: path of the input HDF5 file.
    :type hdf5_file: str
    :param hdf5_group: subgroup within the HDF5 file. Defaults to "/".
    :type hdf5_group: str

    This function is a simple Wrapper around the ``h5toanim`` external command,
    which should be installed for this function to work.

    """
    subprocess.check_call(('h5toanim',
            '--hdf5-file', hdf5_file,
            '--hdf5-group', hdf5_group,
            '--scene-file', collada_scene,
            '--output', collada_animation))

def view(collada_file, hdf5_file=None, hdf5_group="/"):
    """Display a collada file, generating the animation if necessary.

    Usage:

        view(collada_file)
        view(collada_scene_file, hdf5_file, [hdf5_group])

    If only the `collada_file` is given, it is displayed.
    If both `collada_file` and `hdf5_file` are given, they are combined into
    a collada animation file, which is displayed.

    This function is a Wrapper around the ``h5toanim`` and ``daenim`` external
    commands. Tey should both be installed for the function to work.

    """
    if hdf5_file is None:
        subprocess.check_call(('daenim', collada_file))
    else:
        anim_file = tempfile.mkstemp(suffix='anim.dae', text=True)[1]
        write_collada_animation(anim_file, collada_file, hdf5_file, hdf5_group)
        subprocess.check_call(('daenim', anim_file))
        os.remove(anim_file)
