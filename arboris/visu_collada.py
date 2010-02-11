# coding=utf-8

from xml.etree.ElementTree import ElementTree, SubElement, Element, Comment
from numpy import eye, zeros, all, array, ndarray, linspace
from arboris.homogeneousmatrix import inv, rotzyx_angles
import arboris.core
import arboris._visu

def write_collada_scene(world, dae_filename):
    assert isinstance(world, arboris.core.World)
    drawer = arboris._visu.Drawer(ColladaDriver(dae_filename))
    world.parse(drawer)
    drawer.finish()

def write_collada_animation(world, dae_filename, hdf5_filename, hdf5_group="/"):
    import h5py
    assert isinstance(world, arboris.core.World)
    drawer = arboris._visu.Drawer(ColladaDriver(dae_filename))
    world.parse(drawer)
    hdf5_file = h5py.File(hdf5_filename, 'r')
    group = hdf5_file
    for g in hdf5_group.split('/'):
        if g:
            group = group.require_group(g)
    timeline = group['timeline']
    group = group.require_group('transforms')
    #transforms = arboris.core.NamedList(drawer.transforms.keys())
    #for (k, v) in group.iteritems():
    #    node = drawer.transform_nodes[transforms[k]]
    drawer._driver.add_animation(group, timeline)
    drawer.finish()

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

        self.collada = Element("COLLADA", {"version":"1.4.0",
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

    def add_animation(self, transforms, timeline):

        def source(id, param_names, data):
            """Add a 1d source"""
            data = array(data)
            type = None
            if data.dtype.kind == 'S':
                type = 'Name'
            elif data.dtype.kind == 'f':
                type = 'float'
            elif data.dtype.kind == 'i':
                type = 'int'
            elif data.dtype.kind == 'b':
                type = 'bool'
            else:
                raise ValueError('Unknown data')
            array_id = id + "-array"
            stride = len(param_names)
            src = Element("source", {"id":id})
            arr = SubElement(src, type+'_array', {'id':array_id, 'count':str(data.size)})#TODO: use sid instead of id
            arr.text = str(data.reshape(-1)).strip("[]").replace("'", "")
            tch = SubElement(src, "technique_common")
            acs = SubElement(tch, "accessor", 
                    {"source":"#"+array_id,
                     "count":str(data.size/stride),
                     "stride":str(stride)})
            for param_name in param_names:
                par = SubElement(acs, "param", {'type':type})
                if param_name:
                    par.attrib['name'] = param_name
            return src
        
        def sampler(id, input, output, interpolation):
            """Add a sampler and a channel"""
            spl = Element("sampler", {"id":id})
            SubElement(spl,"input", {'semantic':"INPUT", 'source':input})
            SubElement(spl,"input", {'semantic':"OUTPUT", 'source':output})
            SubElement(spl,"input",
                       {'semantic':"INTERPOLATION", 'source':interpolation})
            return spl

        def anim(timeline, transforms, animation_name="myanim"):
            anims = []
            n = len(timeline)
            for node_id, poses in transforms.iteritems():
                # decompose the poses into 4 transforms:
                # a translation and 3 rotations
                # H = T * Rz * Ry * Rz
                transl = poses[:,0:3,3]
                rotz = zeros(n)
                roty = zeros(n)
                rotx = zeros(n)
                for i in range(n):
                    (rotz[i], roty[i], rotx[i]) = rotzyx_angles(poses[i])
                # generate a data source, a sampler and a channel for each
                for transform, param_names, data in (
                        ('translate', ('X','Y','Z'), transl),
                        ('rotateZ', ('ANGLE',), rotz),
                        ('rotateY', ('ANGLE',), roty),
                        ('rotateX', ('ANGLE',), rotx)):
                    id = node_id + '-' + transform
                    sanim = Element('animation', {'id':id})
                    sanim.append(source(id+'-input', ('TIME',), timeline))
                    if transform == 'translate':
                        sanim.append(source(id+'-interpolation',
                                            ('X', 'Y', 'Z'),
                                            array(['STEP']*3*n)))
                        target = node_id+"/"+transform
                    else:
                        sanim.append(source(id+'-interpolation', ('ANGLE',),
                            array(['STEP']*n)))
                        target = node_id+"/"+transform+'.ANGLE'
                    sanim.append(source(id+'-output', param_names, data))
                    sanim.append(sampler(id+'-sampler', '#'+id+'-input', 
                        '#'+id+'-output', '#'+id+'-interpolation'))
                    sanim.append(Element('channel', {'source':'#'+id+'-sampler',
                            'target':target}))
                    anims.append(sanim)
            return anims
        lib = Element("library_animations")
        for a in anim(timeline, transforms):
            lib.append(a)
        self.collada.append(lib)

    def add_child(self, parent, child):
        parent.append(child)

    def create_transform(self, pose, is_constant, name):
        """Generate the node corresponding to the pose."""
        if name:
            node = Element("node", {"id":name, "name":name})
        else:
            node = Element("node")
        if not is_constant:
            matrix = SubElement(node, "translate", {'sid':"translate"})
            matrix.text = "0 0 0"
            matrix = SubElement(node, "rotate", {'sid':"rotateZ"})
            matrix.text = "0 0 1 0.00000"
            matrix = SubElement(node, "rotate", {'sid':"rotateY"})
            matrix.text = "0 1 0 0.00000"
            matrix = SubElement(node, "rotate", {'sid':"rotateX"})
            matrix.text = "1 0 0 0.00000"
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
