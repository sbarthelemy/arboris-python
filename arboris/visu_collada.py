# coding=utf-8
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import XML, Element, SubElement
from numpy import eye, zeros, all, array, ndarray, linspace, pi, linalg
from arboris.homogeneousmatrix import inv, rotzyx_angles, zaligned
import arboris.core
import arboris._visu
import subprocess
import os
import tempfile
from datetime import datetime

NS = 'http://www.collada.org/2005/11/COLLADASchema'
SHAPES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'shapes.dae')

def QN(tag):
    """Return the qualified version of a collada tag name.

    Example:

    >>> QN('asset')
    '{http://www.collada.org/2005/11/COLLADASchema}asset'

    """
    assert tag[1] is not '{'
    return "{" + NS + "}" + tag

def indent(tree):
    """Indent the xml tree."""
    def _indent(elem, level):
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

    _indent(tree.getroot(), 0)

def fix_namespace(tree):
    """Fix a tree to avoid namespace aliases/prefixes.

    This function replaces qualified elements tags from the collada namespace
    by the unqualified (or local) version and add the namespace declaration
    to the root element.

    **Background**

    ColladaCoherencyTest requires the collada file to declare the namespace as
    shown in this example::

        <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"
                 version="1.4.1">
            <asset>
                ...
            </asset>
        </COLLADA>

    Due to this namespace declaration, when ElementTree parses the file, it
    remove the ``xmlns`` attribute from the ``COLLADA`` element and stores
    qualified versions of the tags. For the example ``Element``s' tags would
    be:

    - {http://www.collada.org/2005/11/COLLADASchema}COLLADA
    - {http://www.collada.org/2005/11/COLLADASchema}asset

    There is no way to disable this namespace resolution in ElementTree
    (besides writing a new parser).

    When ElementTree serializes the tree back to an XML file, it generates
    namespace aliases, such as "ns0" for every element.

        <ns0:COLLADA xmlns:ns0="http://www.collada.org/2005/11/COLLADASchema"
                     version="1.4.1">
            <ns0:asset>
                ...
            </ns0:asset>
        </ns0:COLLADA>

    One can customize the prefix, but not disable it (ie. declare a namespace
    to be default.)

    However,collada-dom does not support namespace prefixes and won't load such
    a file. This seems to be a limitation of the whole DTD thing which is not
    namespace-aware.

    In order to write collada-dom compatible files, this function "unqualifies"
    the collada tags and adds back the ``xmlns`` attribute to the root element.

    """
    def _fix_tag(elem):
        if elem.tag.startswith('{'+NS+'}'):
            elem.tag = elem.tag.split('}')[1]
        for elem in elem:
            _fix_tag(elem)
    root = tree.getroot()
    root.set('xmlns', NS)
    _fix_tag(root)

def find_by_id(root, id, tag=None):
    """Find an element by id."""
    for e in root.getiterator(tag):
        if e.get('id') == id:
            return(e)
    return None

class ColladaDriver(arboris._visu.DrawerDriver):
    def __init__(self, filename, shapes_file=None, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale, options)
        self._file = open(filename, 'w')
        self._colors = []
        scene = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                'scene.dae')
        self._tree = ET.parse(scene)
        frame_arrows = find_by_id(self._tree, 'frame_arrows', QN('node'))
        # fix the path to the shapes.dae file
        if shapes_file is None:
            self._shapes = SHAPES
        else:
            self._shapes = shapes_file
        instance_node = frame_arrows.find('./{'+NS+'}instance_node')
        instance_node.set('url', self._shapes + "#frame_arrows")
        # update the frame arrows scale
        scale = frame_arrows.find('./{'+NS+'}scale')
        scale.text = "{0} {0} {0}".format(self._options['frame arrows length'])

    def init(self):
        pass

    def add_ground(self, up):
        self._set_up_axis(up)
        self._set_date()
        ground = find_by_id(self._tree, 'ground', QN('node'))
        assert ground is not None
        return ground

    def _set_up_axis(self, up):
        """Add an up_axis element."""
        if all(up == [1., 0., 0.]):
            up_text = 'X_UP'
        elif all(up == [0., 1., 0.]):
            up_text = 'Y_UP'
        elif all(up == [0., 0., 1.]):
            up_text = 'Z_UP'
        else:
            up_text = None
            warning('the up vector is not compatible with collada')
        if up_text:
            up_element = SubElement(self._tree.find('{'+NS+'}asset'),
                    QN('up_axis'))
            up_element.text = up_text

    def _set_date(self, date=None):
        """Set the created ans modified elements"""
        if date is None:
            date = datetime.now()
        created = self._tree.find('{'+NS+'}asset/{'+NS+'}created')
        created.text = date.isoformat()
        modified = self._tree.find('{'+NS+'}asset/{'+NS+'}modified')
        modified.text = date.isoformat()


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
            node = Element(QN("node"), {"id":name, "name":name})
        else:
            node = Element(QN("node"))
        if not is_constant:
            rotz, roty, rotx = rotzyx_angles(pose)
            coeff = 180./pi
            matrix = SubElement(node, QN("matrix"), {'sid':"matrix"})
            matrix.text = str(pose.reshape(16))[1:-1]
        elif not (pose == eye(4)).all():
            matrix = SubElement(node, QN("matrix"), {'sid':'matrix'})
            matrix.text = str(pose.reshape(-1)).strip('[]')
        return node

    def create_line(self, start, end, color, name=None):
        vector = (array(end) - array(start))
        vector_norm = linalg.norm(vector)
        assert vector_norm > 0.
        n_vector = vector/vector_norm
        H = zaligned(n_vector)
        H[0:3, 3] = start
        node = self.create_transform(H, is_constant=True, name=name)
        scale = SubElement(node, QN('scale'))
        scale.text = "0. 0. {0}".format(vector_norm)
        elem = SubElement(node, QN("instance_geometry"),
            {"url": self._shapes+"#line"})
        self._add_color(elem, color)
        return node

    def create_frame_arrows(self):
        return Element(QN("instance_node"), {"url": "#frame_arrows"})

    def create_box(self, half_extents, color, name=None):
        # instead of creating a new box, we use the #box and scale it
        # to the proper size
        if name:
            node = Element(QN("node"), {"id":name, "name":name})
        else:
            node = Element(QN("node"))
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} {2}".format(*half_extents)
        elem = SubElement(node, QN("instance_geometry"),
                          {"url": self._shapes+"#box"})
        self._add_color(elem, color)
        return node

    def create_plane(self, coeffs, color, name=None):
        H = zaligned(coeffs[0:3])
        H[0:3, 3] = coeffs[3] * coeffs[0:3]
        node = self.create_transform(H, is_constant=True, name=name)
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} 0.".format(*self._options["plane half extents"])
        elem = SubElement(node, QN("instance_geometry"),
                {"url": self._shapes+"#plane"})
        self._add_color(elem, color)
        return node

    def _create_ellipsoid(self, radii, color, resolution, name=None):
        assert resolution in ('20', '80', '320')
        if name:
            node = Element(QN("node"), {"id":name, "name":name})
        else:
            node = Element(QN("node"))
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {1} {2}".format(*radii)
        elem = SubElement(node, QN("instance_geometry"),
                          {"url": self._shapes+"#sphere_"+resolution})
        self._add_color(elem, color)
        return node

    def create_ellipsoid(self, radii, color, name=None):
         return self._create_ellipsoid(radii, color, resolution='320',
                 name=name)

    def create_point(self, color, name=None):
        radii = (self._options['point radius'],) * 3
        return self._create_ellipsoid(radii, color, resolution='20', name=name)

    def _create_cylinder(self, length, radius, color, resolution, name=None):
        assert resolution in ('8', '32')
        if name:
            node = Element(QN("node"), {"id":name, "name":name})
        else:
            node = Element(QN("node"))
        scale = SubElement(node, QN('scale'))
        scale.text = "{0} {0} {1}".format(radius, length)
        elem = SubElement(node, QN("instance_geometry"),
                          {"url": self._shapes+"#cylinder_"+resolution})
        self._add_color(elem, color)
        return node

    def create_cylinder(self, length, radius, color, name=None):
        return self._create_cylinder(length, radius, color, '32', name=name)

    def _color_id(self, color):
        return "color{0}".format(self._colors.index(color))

    def _add_color(self, parent, color):
        if not color in self._colors:
            self._colors.append(color)
        parent.append(XML("""
        <bind_material xmlns="{NS}">
            <technique_common>
                <instance_material symbol="material" target="#{0}" />
            </technique_common>
        </bind_material>
        """.format(self._color_id(color), NS=NS)))

    def _write_colors(self):
        lib_materials = self._tree.find('{'+NS+'}library_materials')
        lib_effects = self._tree.find('{'+NS+'}library_effects')
        for c in self._colors:
            color_id = self._color_id(c)
            lib_materials.append(XML("""
            <material id="{0}" xmlns="{NS}">
                <instance_effect url="#{0}_fx"/>
            </material>
            """.format(color_id, NS=NS)))

            lib_effects.append(XML("""
            <effect id="{0}_fx" xmlns="{NS}">
                <profile_COMMON>
                    <technique sid="blender">
                        <phong>
                            <diffuse>
                                <color>{1} {2} {3} 1</color>
                            </diffuse>
                        </phong>
                    </technique>
                </profile_COMMON>
            </effect>
            """.format(color_id, *c, NS=NS)))

    def finish(self):
        # write to  file
        self._write_colors()
        indent(self._tree)
        fix_namespace(self._tree)
        self._tree.write(self._file, "utf-8")
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

    Usage::

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
