# coding=utf-8

from xml.etree.ElementTree import ElementTree, SubElement, Element, Comment
from numpy import eye, zeros, all, array, ndarray, linspace, pi
from arboris.homogeneousmatrix import inv, rotzyx_angles
import arboris.core
import arboris._visu
import subprocess
import os
import tempfile
tempdir = tempfile.gettempdir()

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

def write_collada_animation(dae_animation, dae_scene, hdf5_filename,
                            hdf5_group="/"):
    """Combine a collada scene and an HDF5 file into a collada animation.

    :param dae_animation: path of the output animation collada file
    :type dae_animation: str
    :param dae_scene: path of the input scene collada file
    :type dae_scene: str
    :param hdf5_filename: path of the input HDF5 file.
    :type hdf5_filename: str
    :param hdf5_group: subgroup within the HDF5 file. Defaults to "/".
    :type hdf5_group: str

    This function is a simple Wrapper around the ``h5toanim`` command, which
    should be installed.

    """
    subprocess.check_call(('h5toanim',
            '--hdf5-file', hdf5_filename,
            '--hdf5-group', hdf5_group,
            '--scene-file', dae_scene,
            '--output', dae_animation))

def view_collada_animation(dae_scene, hdf5_filename, hdf5_group="/"):
    """Display the animation corresponding to a simulation.

    This function is a Wrapper around the ``h5toanim`` and ``daenim`` commands.
    It creates a collada animation file using ``h5toanim`` in a temporary
    location, then calls ``daenim`` to view it.

    Both ``h5toanim`` and ``daenim`` should be installed.

    """
    dae_animation = os.path.join(tempdir, "arboris_animation.dae")
    write_collada_animation(dae_animation, dae_scene, hdf5_filename, hdf5_group)
    subprocess.check_call(('daenim', dae_animation))

def _indent(elem, level=0):
    """Indent the xml subtree starting at elem."""
    istr = " "*8
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

    def __init__(self, filename, scale=1., options=None):
        arboris._visu.DrawerDriver.__init__(self, scale, options)
        self._file = open(filename, 'w')
        self._file.write('<?xml version="1.0" encoding="utf-8"?>\n')

    def init(self):
        pass

    def add_ground(self, up):
        self._init_scene(up)
        return self.visual_scene

    def _init_scene(self, up):

        def asset():
            """Generate the "asset" collada tag."""
            asset = Element("asset")
            SubElement(asset, "created").text = '2005-06-27T21:00:00Z'  #TODO add date
            SubElement(asset, "modified").text = '2005-06-27T21:00:00Z' #TODO add date
            SubElement(asset, "unit", {"meter":"1", "name":"meter"}) #TODO useful ?
            return asset

        def _create_frame_arrows(length):
            """Generate geometry to draw frame arrows."""
            id = 'frame_arrows'
            geom = Element("geometry", {"id":id})
            mesh = SubElement(geom, "mesh")
            source = SubElement(mesh, "source", {"id":id+"_position"})
            array = SubElement(source, "float_array", {"count":"12",
                "id":id+"_position_array"})
            array.text = "0. 0. 0. {0} 0. 0. 0. {0} 0. 0. 0. {0}".format(
                    length)
            tcommon = SubElement(source, "technique_common")
            accessor = SubElement(tcommon, "accessor", {"count":"4",
                "source":'#'+id+'_position_array', "stride":"3"})
            SubElement(accessor, "param", {"type":"float", "name":"X"})
            SubElement(accessor, "param", {"type":"float", "name":"Y"})
            SubElement(accessor, "param", {"type":"float", "name":"Z"})
            vertices = SubElement(mesh, "vertices", {"id":id+"_vertices"})
            SubElement(vertices, "input", {"semantic":"POSITION",
                "source":'#'+id+'_position'})
            lines = SubElement(mesh, "lines", {"count":"3"})
            SubElement(lines, "input", {"semantic":"VERTEX",
                "source":'#'+id+'_vertices'})
            primitive = SubElement(lines, "p")
            primitive.text = "0 1   0 2   0 3"
            return geom

        def _create_box():
            """Generate geometry to draw a box."""
            id = 'box'
            geom = Element("geometry", {"id":id})
            mesh = SubElement(geom, "mesh")
            source = SubElement(mesh, "source", {"id":id+"_position"})
            array = SubElement(source, "float_array", {"count":"24",
                "id":id+"_position_array"})
            array.text = "1 1 -1 1 -1 -1 -1 -1 -1 -1 1 -1 " \
                         "1 1 1 1 -1 1 -1 -1 1 -1 1 1"
            tcommon = SubElement(source, "technique_common")
            accessor = SubElement(tcommon, "accessor", {"count":"8",
                "source":'#'+id+'_position_array', "stride":"3"})
            SubElement(accessor, "param", {"type":"float", "name":"X"})
            SubElement(accessor, "param", {"type":"float", "name":"Y"})
            SubElement(accessor, "param", {"type":"float", "name":"Z"})
            vertices = SubElement(mesh, "vertices", {"id":id+"_vertices"})
            SubElement(vertices, "input", {"semantic":"POSITION",
                "source":'#'+id+'_position'})
            triangles = SubElement(mesh, "triangles", {"count":"12"})
            SubElement(triangles, "input", {"semantic":"VERTEX",
                "source":'#'+id+'_vertices'})
            primitive = SubElement(triangles, "p")
            primitive.text = "0 1 2 2 3 0 4 7 6 6 5 4 0 4 5 5 1 0 1 5 6 6 2 " \
                    "1 2 6 7 7 3 2 4 0 3 3 7 4"
            return geom

        self.collada = Element("COLLADA", {"version":"1.4.1",
                "xmlns":"http://www.collada.org/2005/11/COLLADASchema"})
        self.collada.append(asset())
        #self.collada.append(self._anim()) #TODO: remove
        lib = SubElement(self.collada, "library_geometries")
        lib.append(_create_frame_arrows(self._options['frame arrows length']))
        lib.append(_create_box())
        library_visual_scenes = SubElement(self.collada, "library_visual_scenes")
        scene_name = 'myscene'
        self.visual_scene = SubElement(library_visual_scenes, "visual_scene",
                {'id':scene_name})
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
            asset = SubElement(self.visual_scene, "asset")
            SubElement(asset, "up_axis").text = up
        scene = SubElement(self.collada, "scene")
        SubElement(scene, "instance_visual_scene", {"url":'#'+scene_name})
        return self.visual_scene

    def add_child(self, parent, child):
        parent.append(child)

    def create_transform(self, pose, is_constant, name):
        """Generate the node corresponding to the pose."""
        if name:
            node = Element("node", {"id":name, "name":name})
        else:
            node = Element("node")
        if not is_constant:
            rotz, roty, rotx = rotzyx_angles(pose)
            coeff = 180./pi
            matrix = SubElement(node, "translate", {'sid':"translate"})
            matrix.text = "{0} {1} {2}".format(pose[0, 3],
                                               pose[1, 3],
                                               pose[2, 3])
            matrix = SubElement(node, "rotate", {'sid':"rotateZ"})
            matrix.text = "0 0 1 {0}".format(rotz*coeff)
            matrix = SubElement(node, "rotate", {'sid':"rotateY"})
            matrix.text = "0 1 0 {0}".format(roty*coeff)
            matrix = SubElement(node, "rotate", {'sid':"rotateX"})
            matrix.text = "1 0 0 {0}".format(rotx*coeff)
        elif not (pose == eye(4)).all():
            matrix = SubElement(node, "matrix", {'sid':'matrix'})
            matrix.text = str(pose.reshape(-1)).strip('[]')
        return node

    def create_frame_arrows(self):
        return Element("instance_geometry", {"url":"#frame_arrows"})

    def create_box(self, half_extents, color):
        # instead of creating a new box, we use the #box and scale it
        # to the proper size
        node = Element("node")
        scale = SubElement(node, 'scale')
        scale.text = "{0} {1} {2}".format(*half_extents)
        SubElement(node, "instance_geometry", {"url":"#box"})
        return node

    def finish(self):
        # write to  file
        _indent(self.collada)
        tree = ElementTree(self.collada)
        tree.write(self._file, "utf-8")
        self._file.close()
