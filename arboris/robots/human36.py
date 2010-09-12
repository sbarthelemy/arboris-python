# coding = utf-8

"""
This module serve as a factory for anthropometric humanoid models.

The model is based on the ``human36`` anatomical model from the `HuMAnS
toolbox`_ software, developed at the INRIA in Grenoble.

To the end-user, only the :func:`add_human36` function should be useful.

**Finding the anatomical parameters in the HuMAnS toolbox source code**

HuMAnS (at version 1.0.7) uses Maple-generated C-code for computing
the lagrangian model matrices and performs the integration and contact
resolution in scilab. More precisely, some maple code in the
``HuMAnS/LagrangianModel/Model/Human36/MapleCodeGeneration``  directory
describes

- the lagrangian dynamics,
- the contact kinematics,
- and the "tags" kinematics (in the HuMAnS vocable, Tags are
  feature points such as the anatomical landmarks used for human
  motion reconstruction, as defined as "Sites" in H-anim).

These files generate the C files that are then called from scilab. The functions
take as parameters the human state (generalized positions and velocities) and
anatomical parameters which are divided in

- parameters needed for kinematic model (the ``L`` variable in
  ``HuMAnS/LagrangianModel/Human36/MapleCodeGeneration/kinematicData.maple``),

- parmeters needed for motion reconstruction and contacts (the ``AddL``
  variable in
  ``HuMAnS/LagrangianModel/Human36/MapleCodeGeneration/AdditionnalData.maple``).

The two arrays of parameters are computed from the human's total height in the
function ``SetModelSize`` from the file
``HuMAnS/LagrangianModel/Human36/human36.c``.

The data about mass geomerty are located in
``HuMAnS/LagrangianModel/Model/Human36/MapleCodeGeneration/DynamicData.maple``.

Warning: the HuMAnS doc about inertia computation was erroneous (the real math
is in the ``IOMatrix`` proc in ``DynamicData.maple``).

.. _`HuMAnS toolbox`:
    http://bipop.inrialpes.fr/software/humans/
    Human Motion Analysis and Simulation toolbox

"""
from arboris.core import World, Body, SubFrame, NamedObjectsList
import numpy as np
from numpy import array, diag, dot, hstack
import arboris.homogeneousmatrix as Hg
from arboris.homogeneousmatrix import adjoint
from arboris.joints import *
from arboris.shapes import Point

def anat_lengths_from_height(height):
    """Dict-like object storing anatomical lengths as defined in HuMAnS.

    :param height: the human height
    :type height: float
    :return: anatomical lengths, scaled according to ``height``
    :rtype: dict

    **Examples:**

    >>> L = anat_lengths_from_height(1.741)
    >>> L['ysternoclavL']
    0.17061800000000002
    >>> L = anat_lengths_from_height(2.)
    >>> L['ysternoclavL']
    0.19600000000000001

    ** Differences with HuMAnS **

    The data comes from the file ``LagrangianModel/Human36/Human36.c``
    in HuMAnS.
    The lengths defined in this class correspond to the ``L`` variable in
    ``HuMAnS/LagrangianModel/Human36/MapleCodeGeneration/kinematicData.maple``,
    with the addition of ``xheelR`` and ``xheelL`` lenghts are not defined
    in ``anatomicalLengths`` but in ``tagMultSize``

    ** Differences with matlab-arboris **

    In this implementation, the "zshoulder", "zhip" and "xfoot" lengths are
    computed as in HuMAnS, which is different from the way they were
    computed in matlab-arboris.

    """
    lengths = {}
    def add_length(name, value, descr=None):
        """Add an anatomical length.

        :param name: the length name, taken from HuMAnS when possible
        :type name: string
        :param value: length value, in meters
        :type value: float
        :param descr: a short descriptive text
        :type descr: string or None

        ``descr`` is discarded for now.

        """
        name = unicode(name)
        lengths[name] = float(value)

    yfoot =  0.0222 * height
    ytibia = 0.2493 * height
    yfemur = 0.2425 * height
    ysternoclav = 0.0980 * height
    zsternoclav = 0.5 * 0.0254 # zsternoclav = 0.5 inch
    xsternoclav = 0.1052 *  height
    yshoulder = 0.0104 * height
    zshoulder = 0.1295 * height - zsternoclav
    xshoulder = 0.0526 * height
    yhumerus = 0.1618 * height
    yforearm = 0.1544 * height
    yhand = 0.1091 * height
    xfoot = 0.1482 * height
    xheel = 0.0248 * height
    add_length("yvT10", 0.2075 * height, "hip joint centers/T10 vertebra")
    add_length("xvT10", 0.0526 * height, "hip joint center middle point/T10 vertebra")
    add_length("zhip", 0.1002 * height, "left hip joint center/right hip joint center")
    add_length("yfootR", yfoot, "right foot height")
    add_length("ytibiaR", ytibia, "right lateral malleolus/right knee joint center")
    add_length("yfemurR", yfemur, "right knee Joint center/right hip joint center")
    add_length("ysternoclavR", ysternoclav, "T10 vertebra/suprasternale")
    add_length("zsternoclavR", zsternoclav)
    add_length("xsternoclavR", xsternoclav, "T10 vertebra/right sternoclavicular joint center")
    add_length("yshoulderR", yshoulder,  "suprasternale/right shoulder joint center")
    add_length("zshoulderR", zshoulder,  "suprasternale/right shoulder joint center")
    add_length("xshoulderR", xshoulder,  "suprasternale/right shoulder joint center")
    add_length("yhumerusR", yhumerus,  "right shoulder joint center/right elbow joint center")
    add_length("yfootL", yfoot, "left foot height")
    add_length("ytibiaL", ytibia, "left lateral malleolus/left knee joint center")
    add_length("yfemurL", yfemur, "left knee Joint center/left hip joint center")
    add_length("ysternoclavL", ysternoclav, "T10 vertebra/suprasternale")
    add_length("zsternoclavL", zsternoclav, )
    add_length("xsternoclavL", xsternoclav, "T10 vertebra/left sternoclavicular joint center")
    add_length("yshoulderL", yshoulder, "suprasternale/left shoulder joint center")
    add_length("zshoulderL", zshoulder, "suprasternale/left shoulder joint center")
    add_length("xshoulderL", xshoulder, "suprasternale/left shoulder joint center")
    add_length("yhumerusL", yhumerus, "left shoulder joint center/left elbow joint center")
    add_length("yforearmR", yforearm, "right elbow joint center/right wrist joint center")
    add_length("yforearmL", yforearm, "left elbow joint center/left wrist joint center")
    add_length("yvC7", 0.139 * height, "T10 vertebra/C7 vertebra")
    add_length("yhandR", yhand, "right wrist joint center/right 3rd dactilion")
    add_length("yhandL", yhand, "left wrist joint center/left 3rd dactilion")
    add_length("yhead", 0.1395 * height, "C7 vertebra/Vertex")
    add_length("xfootR", xfoot,  "right foot length")
    add_length("xfootL", xfoot, "left foot length")
    add_length("xheelR", xheel, 'right ankle/right heel')
    add_length("xheelL", xheel, 'left ankle/left heel')
    return lengths

def height_from_anat_lengths(lengths):
    """Computes a human height according to its anatomical lengths.

    An exception is thrown if the human is asymmetric (one leg longer than the
    other...).

    :return height: the human height
    :rtype: float
    :raises: ValueError if one leg is longer than the other

    **Examples:**

    >>> L = anat_lengths_from_height(1.741)
    >>> height_from_anat_lengths(L)
    1.7410000000000001
    >>> L['ytibiaL'] = .95 * L['ytibiaL'] # shorten the left leg
    >>> height_from_anat_lengths(L)
    Traceback (most recent call last):
        ...
    ValueError: The legs have different lengths

    """
    right_leg = lengths['yfootR'] + lengths['ytibiaR'] + lengths['yfemurR']
    left_leg = lengths['yfootL'] + lengths['ytibiaL'] + lengths['yfemurL']
    if left_leg != right_leg:
        raise ValueError("The legs have different lengths")
    return left_leg + lengths['yvT10'] + lengths['yvC7'] + lengths['yhead']


def add_human36(world, height=1.741, mass=73, anat_lengths=None, name='',
                return_lists=False):
    """Add an anthropometric humanoid model to the world.

    :param height: the human height in meters. Ignored if ``anat_lengths``
                   is provided.
    :type height: float
    :param mass: the human mass in kilograms.
    :type mass: float
    :param anat_lengths: the human anatomical lengths.
                         Computed from ``height`` if not provided
    :type anat_lengths: dict
    :param name: name of the human, used to prefix every object name.
    :type name: string
    :param return_lists: if True, returns of a tuple of the added objects
    :type return_lists: boolean
    :return: None or a tuple of ``NamedObjectsList``s of the added objects
             ``(joints, bodies, tags, shapes)``

    **Exemples**

    >>> w = World()
    >>> # add a normal human
    >>> add_human36(w, height=1.8, name="Bob's ")
    >>> # add a human with a shorter left arm
    >>> L = anat_lengths_from_height(1.8)
    >>> L['yhumerusL'] *= .7
    >>> L['yforearmL'] *= .7
    >>> L['yhandL'] *= .7
    >>> add_human36(w, anat_lengths=L, name="Casimodo's ")
    >>> frames = w.getframes()

    >> frames["Bob's Left stylion"].bpose[0:3, 3]
    >> frames["Bob's Left stylion"].pose[0:3, 3]
    >> frames["Casimodo's Left stylion"].bpose[0:3, 3]

    """
    assert isinstance(world, World)
    w = world
    if anat_lengths is None:
        L = anat_lengths_from_height(height)
    else:
        L = anat_lengths
    h = height_from_anat_lengths(L)
    prefix = name

    # create the bodies
    bodies = NamedObjectsList()
    def add_body(name, mass, com_position, gyration_radius):
        #mass matrix at com
        mass_g = mass * diag(hstack((gyration_radius**2, (1,1,1))))
        H_fg = eye(4)
        H_fg[0:3,3] = com_position
        H_gf = Hg.inv(H_fg)
        #mass matrix at body's frame origin:
        mass_o = dot(adjoint(H_gf).T, dot(mass_g, adjoint(H_gf)))
        if name:
            name = prefix + name
        bodies.append(Body(name=name, mass=mass_o))
    # Lower Part of Trunk
    add_body("LPT", 0.275 * mass,
             [0, 0.5108*L['yvT10'], 0],
             array([0.2722, 0.2628, 0.226])*L['yvT10'])
    add_body("ThighR", 0.1416 * mass,
             [0, -0.4095*L['yfemurR'], 0],
              array([0.329, 0.149, 0.329])*L['yfemurR'])
    add_body("ShankR", 0.0433 * mass,
             [0, -0.4459*L['ytibiaR'], 0],
             array([0.255, 0.103, 0.249])*L['ytibiaR'])
    add_body("FootR", 0.0137 * mass,
             [0.4415*L['xfootR'] - L['xheelR'], -L['yfootR']/2., 0.],
             array([0.124, 0.257, 0.245])*L['xfootR'])
    add_body("ThighL", 0.1416 * mass,
             [0, -0.4095*L['yfemurL'], 0],
              array([0.329, 0.149, 0.329])*L['yfemurL'])
    add_body("ShankL", 0.0433 * mass,
             [0, -0.4459*L['ytibiaL'], 0],
             array([0.255, 0.103, 0.249])*L['ytibiaL'])
    add_body("FootL", 0.0137 * mass,
             [0.4415*L['xfootL'] - L['xheelL'], -L["yfootL"]/2, 0.],
             array([0.124, 0.257, 0.245])*L['xfootL'])
    # Upper Part of Trunk
    add_body("UPT", 0.1596 * mass,
             [(L['xsternoclavR'] + L['xsternoclavL'])/4.,
              0.7001*(L['ysternoclavR']+L['ysternoclavL'])/2.,
              0.],
             array([0.716, 0.659, 0.454])*L['ysternoclavR'])
    # right shoulder
    add_body("ScapulaR", 0.,
             [0., 0., 0.],
             array([0., 0., 0.]))
    add_body("ArmR", 0.0271 * mass,
             [0., -0.5772*L['yhumerusR'], 0.],
             array([0.285, 0.158, 0.269])*L['yhumerusR'])
    add_body("ForearmR", 0.0162 * mass,
             [0., -0.4574*L['yforearmR'], 0.],
             array([0.276, 0.121, 0.265])*L['yforearmR'])
    add_body("HandR", 0.0061 * mass,
             [0, -0.3691*L['yhandR'], 0],
             array([0.235, 0.184, 0.288])*L['yhandR'])
    # left shoulder
    add_body("ScapulaL", 0.,
             [0., 0., 0.],
             array([0., 0., 0.]))
    add_body("ArmL", 0.0271 * mass,
             [0, -0.5772*L['yhumerusL'], 0.],
             array([0.285, 0.158, 0.269])*L['yhumerusL'])
    add_body("ForearmL", 0.0162 * mass,
             [0, -0.4574*L['yforearmL'], 0],
             array([0.276, 0.121, 0.265])*L['yforearmL'])
    add_body("HandL", 0.0061 * mass,
             [0, -0.3691*L['yhandL'], 0],
             array([0.288, 0.184, 0.235])*L['yhandL'])
    add_body("Head", 0.0694 * mass,
             [0, 0.4998*L['yhead'], 0],
             array([0.303, 0.261, 0.315])*L['yhead'])

    # create the joints and add the links (i.e. joints+bodies)
    joints = NamedObjectsList()
    def add_link(body0, transl, joint, body1):
        if not isinstance(body0, Body):
            body0 = bodies[prefix+body0]
        frame0 = SubFrame(body0, Hg.transl(*transl))
        joints.append(joint)
        w.add_link(frame0, joint, bodies[prefix+body1])

    add_link(w.ground, (0, L['yfootL']+L['ytibiaL']+L['yfemurL'], 0),
             FreeJoint(), 'LPT')
    add_link('LPT', (0, 0, L['zhip']/2.),
             RzRyRxJoint(), 'ThighR')
    add_link('ThighR', (0, -L['yfemurR'], 0),
             RzJoint(), 'ShankR')
    add_link('ShankR', (0, -L['ytibiaR'], 0),
             RzRxJoint(), 'FootR')
    add_link('LPT', (0, 0, -L['zhip']/2.),
             RzRyRxJoint(), 'ThighL')
    add_link('ThighL', (0, -L['yfemurL'], 0),
             RzJoint(), 'ShankL')
    add_link('ShankL', (0, -L['ytibiaL'], 0),
             RzRxJoint(), 'FootL')
    add_link('LPT', (-L['xvT10'], L['yvT10'], 0),
             RzRyRxJoint(), 'UPT')
    add_link('UPT', (L['xsternoclavR'], L['ysternoclavR'], L['zsternoclavR']),
             RyRxJoint(), 'ScapulaR')
    add_link('ScapulaR', (-L['xshoulderR'], L['yshoulderR'], L['zshoulderR']),
             RzRyRxJoint(), 'ArmR')
    add_link('ArmR', (0, -L['yhumerusR'], 0),
             RzRyJoint(), 'ForearmR')
    add_link('ForearmR', (0, -L['yforearmR'], 0),
             RzRxJoint(), 'HandR')
    add_link('UPT', (L['xsternoclavL'], L['ysternoclavL'], -L['zsternoclavL']),
             RyRxJoint(), 'ScapulaL')
    add_link('ScapulaL', (-L['xshoulderL'], L['yshoulderL'], -L['zshoulderL']),
             RzRyRxJoint(), 'ArmL')
    add_link('ArmL', (0, -L['yhumerusL'], 0),
             RzRyJoint(), 'ForearmL')
    add_link('ForearmL', (0, -L['yforearmL'], 0),
             RzRxJoint(), 'HandL')
    add_link('UPT', (L['xvT10'], L['yvC7'], 0),
             RzRyRxJoint(), 'Head')

    # add the tags
    tags = NamedObjectsList()
    def add_tag(name, body, position):
        """Returns data about anatomical landmarks as defined in HuMAnS.

        :param name: name of the tag (copied from HuMAnS)
        :param body: name of the body in whose frame the point is defined
        :position: tag coordinates in meters

        """
        if name:
            name = prefix + name
        tag = SubFrame(bodies[prefix+body], Hg.transl(*position), name)
        tags.append(tag)
        w.register(tag)
    # we add 1e-4*h to keep the compatibility with HuMAnS:
    add_tag('Right foot toe tip', 'FootR', [L['xfootR']-L['xheelR']+1e-4*h, -L['yfootR'], 0.])
    add_tag('Right foot heel', 'FootR', [-L['xheelR'], -L['yfootR'],0.])
    add_tag('Right foot phalange 5', 'FootR', [0.0662*h, -L['yfootR'], 0.0305*h] )
    add_tag('Right foot Phalange 1', 'FootR', [0.0662*h, -L['yfootR'], -0.0305*h])
    add_tag('Right foot lateral malleolus', "ShankR", [0., -L['ytibiaR'], 0.0249*h])
    add_tag('Femoral lateral epicondyle', "ThighR", [0., -L['yfemurR'], 0.0290*h])
    add_tag('Right great trochanter', "ThighR", [0., 0., 0.0941*h-L['zhip']/2.])
    add_tag('Right iliac crest', "LPT", [0.0271*h, 0.0366*h, 0.0697*h])
    # we add 1e-4*h to keep the compatibility with HuMAnS:
    add_tag('Left foot toe tip', "FootL", [L['xfootL']-L['xheelL']+1e-4*h, -L['yfootL'], 0.])
    add_tag('Left foot heel', "FootL", [-L['xheelL'], -L['yfootL'], 0.])
    add_tag('Left foot phalange 5', "FootL", [0.0662*h, -L['yfootL'], -0.0305*h])
    add_tag('Left foot phalange 1', "FootL", [0.0662*h, -L['yfootL'], 0.0305*h])
    add_tag('Left foot lateral malleolus', "ShankL", [0, -L['ytibiaL'], -0.0249*h])
    add_tag('Left femoral lateral epicondyle', "ThighL", [0,-L['yfemurL'], -0.0290*h])
    add_tag('Left great trochanter', "ThighL",  [0, 0, -0.0941*h+L['zhip']/2.])
    add_tag('Left iliac crest', "LPT", [0.0271*h, 0.0366*h, -0.0697*h])
    add_tag('Substernale (Xyphoid)', "UPT", [0.1219*h, 0, 0])
    add_tag('Suprasternale', "UPT", [(L['xsternoclavL']+L['xsternoclavL'])/2., (L['ysternoclavL']+L['ysternoclavL'])/2., 0])
    add_tag('Right acromion', "ScapulaR", [-L['xshoulderR'], 0.0198*h+L['yshoulderR'], L['zshoulderR']])
    add_tag('Right humeral lateral epicondyle (radiale)', "ArmR", [0., -L['yhumerusR'], 0.0211*h])
    add_tag('Right stylion', "ForearmR", [0., -0.1533*h, 0.0331*h])
    add_tag('Right 3rd dactylion', "HandR", [0., -L['yhandR'], 0.])
    add_tag('Left acromion', "ScapulaL", [-L['xshoulderL'], 0.0198*h+L['yshoulderL'], -L['zshoulderL']])
    add_tag('Left humeral lateral epicondyle (radiale)', "ArmL", [0., -L['yhumerusL'], -0.0211*h])
    add_tag('Left stylion', "ForearmL", [0., -0.1533*h, -0.0331*h])
    add_tag('Left 3rd dactylion', "HandL", [0., -L['yhandL'], 0.])
    add_tag('Cervicale', "UPT", [-0.0392*0. +L['xvT10'], L['yvC7'], 0.])
    add_tag('Vertex', "Head", [0., L["yhead"], 0.])

    shapes = NamedObjectsList()
    # Add point shapes to the feet
    for k in ('Right foot toe tip', 'Right foot heel',
             'Right foot phalange 5', 'Right foot Phalange 1',
             'Left foot toe tip','Left foot heel',
              'Left foot phalange 5','Left foot phalange 1'):
        name = prefix + k
        shape = Point(tags[name], name=name)
        shapes.append(shape)
        w.register(shape)

    w.init()
    if return_lists:
        return (joints, bodies, tags, shapes)
    else:
        return None
