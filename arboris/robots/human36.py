# coding = utf-8

"""
This module serve as a factory for anthropometric humanoid models.

The model is based on the ``human36`` anatomical model from the `HuMAnS
toolbox`_ software developed at the INRIA in Grenoble.

To the end-user, only the :func:`human36` should be useful.

TODO: decide if we support or no the ArborisName field in tags
TODO: fix errors in matlab-arboris

Finding the anatomical parameters in the HuMAnS toolbox source code
===================================================================

HuMAnS (at version 1.0.7) uses Maple-generated C-code for computing
the lagrangian model matrices and performs the integration and contact
resolution in scilab. More precisely, some maple code in the
``HuMAnS/LagrangianModel/Model/Human36/MapleCodeGeneration``  directory
describes

- the lagrangian dynamics,
- the contact kinematics,
- and the "tags" kinematics (in the HuMAnS vocable, Tags are
  caracteristic points such as the anatomical landmarks used for human
  motion reconstruction).

These files generate the C files that are then called from scilab. The functions
take as parameters the human state (generalized positions and velocities) and
anatomical parameters which are divided in

- parameters nedded for kinematic model (the ``L`` variable in
  ``HuMAnS/LagrangianModel/Human36/MapleCodeGeneration/kinematicData.maple``),

- parmeters needed for motion reconstruction and contacts (the ``AddL``
  variable in
  ``HuMAnS/LagrangianModel/Human36/MapleCodeGeneration/AdditionnalData.maple``).

The two arrays of parameters are computed from the human's total height in the
function ``SetModelSize`` from the file
``HuMAnS/LagrangianModel/Human36/human36.c``.

.. _`HuMAnS toolbox`:
    http://bipop.inrialpes.fr/software/humans/
    Human Motion Analysis and Simulation toolbox

"""
from arboris.core import World, Body, SubFrame
import numpy as np
from numpy import array, diag, dot, hstack
import arboris.homogeneousmatrix as Hg
from arboris.homogeneousmatrix import adjoint
from arboris.joints import *
from arboris.shapes import Point

def anatomical_lengths(height):
    """Returns a dict of anatomical lengths as defined in HuMAnS.

    :param height: the human height
    :type height: float
    :return: anatomical lengths, scaled according to ``height``
    :rtype: dict

    This function simply calls :func:`_humans_anatomical_lengths`
    and removes the useless stuff from it result.

    **Examples:**

    >>> L = anatomical_lengths(1.741)
    >>> L['ysternoclavL']
    0.17061800000000002
    >>> L = anatomical_lengths(2.)
    >>> L['ysternoclavL']
    0.19600000000000001

    """
    lengths_list = _humans_anatomical_lengths(height)
    lengths_dict = {}
    for length in lengths_list:
        lengths_dict[length["HumansName"]] = length["Value"]
    return lengths_dict

def _humans_anatomical_lengths(height):
    """Returns data about anatomical lengths as defined in HuMAnS.

    :param height: the human height
    :type height: float
    :return: anatomical lengths date, scaled according to ``height``.
    The keys are:
        - "HumansName": the name used in HuMAnS to denote a parameter,
        - "HumansId": the number used in HuMAnS doc to denote a parameter,
        - "Value": the length value in meters,
        - "Description": a short descriptive text.
    :rtype: list of dicts

    the data come from HuMAnS' ``LagrangianModel/Human36/Human36.c``
    file.

    **Errors in matlab-arboris ?**

    In this implementation, the "zshoulder", "zhip" and "xfoot" lengths are
    computed as in HuMAnS, which is different from the way they were
    computed in matlab-arboris.

    """

    yfoot =  0.0222 * height
    ytibia = 0.2493 * height
    yfemur = 0.2425 * height
    ysternoclav = 0.0980 * height
    zsternoclav = 0.5 * 0.0254 # zsternoclav  = 0.5 inch
    xsternoclav = 0.1052 *  height
    yshoulder = 0.0104 * height
    zshoulder = 0.1295 * height - zsternoclav
    xshoulder = 0.0526 *  height
    yhumerus = 0.1618 * height
    yforearm = 0.1544 * height
    yhand = 0.1091 * height
    xfoot = 0.1482 * height

    lengths = []
    lengths.append({
        "HumansName": "yvT10",
        'HumansId': 1,
        "Value": 0.2075*height,
        "Description": "hip joint centers/T10 vertebra"})
    lengths.append({
        "HumansName": "xvT10",
        'HumansId': 2,
        "Value": 0.0526 *  height,
        "Description": "hip joint center middle point/T10 vertebra"})
    lengths.append({
        "HumansName": "zhip",
        'HumansId': 3,
        "Value": 0.1002 * height,
        "Description": "left hip joint center/right hip joint center"})
    lengths.append({
        "HumansName": "yfootR",
        'HumansId': 4,
        "Value": yfoot,
        "Description": "right foot height"})
    lengths.append({
        "HumansName": "ytibiaR",
        'HumansId': 5,
        "Value": ytibia,
        "Description": "right lateral malleolus/right knee joint center"})
    lengths.append({
        "HumansName": "yfemurR",
        'HumansId': 6,
        "Value": yfemur,
        "Description": "right knee Joint center/right hip joint center"})
    lengths.append({
        "HumansName": "ysternoclavR",
        'HumansId': 7,
        "Value": ysternoclav,
        "Description": "T10 vertebra/suprasternale"})
    lengths.append({
        "HumansName": "zsternoclavR",
        'HumansId': 8,
        "Value": zsternoclav,
        "Description": "?"})
    lengths.append({
        "HumansName": "xsternoclavR",
        'HumansId': 9,
        "Value": xsternoclav,
        "Description": "T10 vertebra/right sternoclavicular joint center"})
    lengths.append({
        "HumansName": "yshoulderR",
        'HumansId': 10,
        "Value": yshoulder,
        "Description": "suprasternale/right shoulder joint center"})
    lengths.append({
        "HumansName": "zshoulderR",
        'HumansId': 11,
        "Value": zshoulder,
        "Description": "suprasternale/right shoulder joint center"})
    lengths.append({
        "HumansName": "xshoulderR",
        'HumansId': 12,
        "Value": xshoulder,
        "Description": "suprasternale/right shoulder joint center"})
    lengths.append({
        "HumansName": "yhumerusR",
        'HumansId': 13,
        "Value": yhumerus,
        "Description": "right shoulder joint center/right elbow joint center"})
    lengths.append({
        "HumansName": "yfootL",
        'HumansId': 14,
        "Value": yfoot,
        "Description": "left foot height"})
    lengths.append({
        "HumansName": "ytibiaL",
        'HumansId': 15,
        "Value": ytibia,
        "Description": "left lateral malleolus/left knee joint center"})
    lengths.append({
        "HumansName": "yfemurL",
        'HumansId': 16,
        "Value": yfemur,
        "Description": "left knee Joint center/left hip joint center"})
    lengths.append({
        "HumansName": "ysternoclavL",
        'HumansId': 17,
        "Value": ysternoclav,
        "Description": "T10 vertebra/suprasternale"})
    lengths.append({
        "HumansName": "zsternoclavL",
        'HumansId': 18,
        "Value": zsternoclav,
        "Description": "?"})
    lengths.append({
        "HumansName": "xsternoclavL",
        'HumansId': 19,
        "Value": xsternoclav,
        "Description": "T10 vertebra/left sternoclavicular joint center"})
    lengths.append({
        "HumansName": "yshoulderL",
        'HumansId': 20,
        "Value": yshoulder,
        "Description": "suprasternale/left shoulder joint center"})
    lengths.append({
        "HumansName": "zshoulderL",
        'HumansId': 21,
        "Value": zshoulder,
        "Description": "suprasternale/left shoulder joint center"})
    lengths.append({
        "HumansName": "xshoulderL",
        'HumansId': 22,
        "Value": xshoulder,
        "Description": "suprasternale/left shoulder joint center"})
    lengths.append({
        "HumansName": "yhumerusL",
        'HumansId': 23,
        "Value": yhumerus,
        "Description": "left shoulder joint center/left elbow joint center"})
    lengths.append({
        "HumansName": "yforearmR",
        'HumansId': 24,
        "Value": yforearm,
        "Description": "right elbow joint center/right wrist joint center"})
    lengths.append({
        "HumansName": "yforearmL",
        'HumansId': 25,
        "Value": yforearm,
        "Description": "left elbow joint center/left wrist joint center"})
    lengths.append({
        "HumansName": "yvC7",
        'HumansId': 26,
        "Value": 0.139*height,
        "Description": "T10 vertebra/C7 vertebra"})
    lengths.append({
        "HumansName": "yhandR",
        'HumansId': 27,
        "Value": yhand,
         "Description": "right wrist joint center/right 3rd dactilion"})
    lengths.append({
        "HumansName": "yhandL",
        'HumansId': 28,
        "Value": yhand,
        "Description": "left wrist joint center/left 3rd dactilion"})
    lengths.append({
        "HumansName": "yhead",
        'HumansId': 29,
        "Value": 0.1395*height,
        "Description": "C7 vertebra/Vertex"})
    lengths.append({
        "HumansName": "xfootR",
        'HumansId': 30,
        "Value": xfoot,
        "Description": "right foot length"})
    lengths.append({
        "HumansName": "xfootL",
        'HumansId': 31,
        "Value": xfoot,
        "Description": "left foot length"})
    return lengths

def height(lengths):
    """Computes a human height according to its anatomical lenghts.

    An exception is thrown if the human is asymetric (one leg longuer than the
    other...).

    **Examples:**

    >>> height(anatomical_lengths(1.741))
    1.7410000000000001
    >>> height(anatomical_lengths(2.))
    2.0

    """
    right_leg = lengths['yfootR'] + lengths['ytibiaR'] + lengths['yfemurR']
    left_leg = lengths['yfootL'] + lengths['ytibiaL'] + lengths['yfemurL']
    if left_leg != right_leg:
        raise ValueError("The legs have different lengths")
    return left_leg + lengths['yvT10'] + lengths['yvC7'] + lengths['yhead']

def _humans_tags(height):
    """Returns data about anatomical landmarks as defined in HuMAnS.

    :param height: the human height
    :type height: float
    :return: anatomical landmarks data, scaled according to ``height``.
    The keys are:
        - "ArborisName": the name used in arboris-matlab to denote the landmark,
        - "HumansName": the name used in HuMAnS to denote the landmark,
        - "HumansId": the number used in HuMAnS doc to denote the landmark,
        - "HumansBodyId": the number used in HuMAnS to denote the body the
          landmark is attached to,
        - "Position": the length value in meters.
    :rtype: list of dicts

    - ``HumansName`` and ``HumansId`` come from
      ``AdditionnalData.maple``.
    - ``Position`` refers to ``AddL`` and come from ``Humans36.c``

    """
    h = height
    L = anatomical_lengths(h)
    tags= []
    tags.append({
        'ArborisName': 'RightFootSecondDigit',
        'HumansName': 'Right foot toe tip',
        'HumansId': 1,
        'HumansBodyId': 4,
        'Position': [0.1235*h, -L['yfootR'], 0.]})
    tags.append({
        'ArborisName': 'RightCalcaneousPost',
        'HumansName': 'Right foot heel',
        'HumansId' : 2,
        'HumansBodyId': 4,
        'Position': [-0.0248*h, -L['yfootR'],0.]})
    tags.append({
        'ArborisName': 'RightFifthMetatarsal',
        'HumansName': 'Right foot phalange 5',
        'HumansId': 3,
        'HumansBodyId': 4,
        'Position': [0.0662*h, -L['yfootR'], 0.0305*h] })
    tags.append({
        'ArborisName': 'RightFirstMetatarsal',
        'HumansName': 'Right foot Phalange 1',
        'HumansId': 4,
        'HumansBodyId': 4,
        'Position': [0.0662*h, -L['yfootR'], -0.0305*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right foot lateral malleolus',
        'HumansId': 5,
        'HumansBodyId': 3,
        'Position': [0., -L['ytibiaR'], 0.0249*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Femoral lateral epicondyle',
        'HumansId': 6,
        'HumansBodyId': 2,
        'Position': [0., -L['yfemurR'], 0.0290*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right great trochanter',
        'HumansId': 7,
        'HumansBodyId': 2,
        'Position': [0., 0., 0.0941*h-L['zhip']/2.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right iliac crest',
        'HumansId': 8,
        'HumansBodyId': 1,
        'Position': [0.0271*h, 0.0366*h, 0.0697*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left foot toe tip',
        'HumansId': 9,
        'HumansBodyId': 7,
        'Position': [0.1235*h, -L['yfootL'], 0.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left foot heel',
        'HumansId': 10,
        'HumansBodyId': 7,
        'Position': [-0.0248*h, -L['yfootL'], 0.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left foot phalange 5',
        'HumansId': 11,
        'HumansBodyId': 7,
        'Position': [0.0662*h, -L['yfootL'], -0.0305*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left foot phalange 1',
        'HumansId': 12,
        'HumansBodyId': 7,
        'Position': [0.0662*h, -L['yfootL'], 0.0305*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left foot lateral malleolus',
        'HumansId': 13,
        'HumansBodyId': 6,
        'Position': [0, -L['ytibiaL'], -0.0249*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left femoral lateral epicondyle',
        'HumansId': 14,
        'HumansBodyId': 5,
        'Position':  [0,-L['yfemurL'], -0.0290*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left great trochanter',
        'HumansId': 15,
        'HumansBodyId': 5,
        'Position':  [0, 0, -0.0941*h+L['zhip']/2.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left iliac crest',
        'HumansId': 16,
        'HumansBodyId': 1,
        'Position': [0.0271*h, 0.0366*h, -0.0697*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Substernale (Xyphoid)',
        'HumansId': 17,
        'HumansBodyId': 8,
        'Position': [0.1219*h, 0, 0]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Suprasternale',
        'HumansId': 18,
        'HumansBodyId': 8,
        'Position': [(L['xsternoclavL']+L['xsternoclavL'])/2.,
                     (L['ysternoclavL']+L['ysternoclavL'])/2.,
                     0]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right acromion',
        'HumansId': 19,
        'HumansBodyId': 9,
        'Position': [-L['xshoulderR'],
                     0.0198*h+L['yshoulderR'],
                     L['zshoulderR']]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right humeral lateral epicondyle (radiale)',
        'HumansId': 20,
        'HumansBodyId': 10,
        'Position': [0., -L['yhumerusR'], 0.0211*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right stylion',
        'HumansId': 21,
        'HumansBodyId': 11,
        'Position': [0., -0.1533*h, 0.0331*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Right 3rd dactylion',
        'HumansId': 22,
        'HumansBodyId': 12,
        'Position': [0., -0.1091*h, 0.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left acromion',
        'HumansId': 23,
        'HumansBodyId': 13,
        'Position': [-L['xshoulderL'], 0.0198*h+L['yshoulderL'], -L['zshoulderL']]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left humeral lateral epicondyle (radiale)',
        'HumansId': 24,
        'HumansBodyId': 14,
        'Position': [0., -L['yhumerusL'], -0.0211*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left stylion',
        'HumansId': 25,
        'HumansBodyId': 15,
        'Position': [0., -0.1533*h, -0.0331*h]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Left 3rd dactylion',
        'HumansId': 26,
        'HumansBodyId': 16,
        'Position': [0., -0.1091*h, 0.]})
    tags.append({
        'ArborisName': '',
        'HumansName': 'Cervicale',
        'HumansId': 27,
        'HumansBodyId': 8,
        'Position': [-0.0392*0. +L['xvT10'], L['yvC7'], 0.]})#79???
    tags.append({
        'ArborisName': '',
        'HumansName': 'Vertex',
        'HumansId': 28,
        'HumansBodyId': 17,
        'Position': [0., 0.1395*h, 0.]})
    return tags


def tags(height):
    """

    """
    tdict = {}
    for t in _humans_tags(height):
        tdict[t['HumansName']] =  t['Position']
    return tdict


def _humans_bodies(height, mass):
    """

    Example:

    >>> b = _humans_bodies(1.741, 73)
    """
    L = anatomical_lengths(height)
    T = tags(height)
    bodies=[]
    bodies.append(
        {"HumansName": "LPT", # Lower Part of Trunk
         'HumansId': 1,
         "Mass": 0.275 * mass,
         "CenterOfMass": [0, 0.5108*L['yvT10'], 0],
         "GyrationRadius": array([0.2722, 0.2628, 0.226])*L['yvT10']})
    bodies.append(
        {"HumansName": "ThighR",
         'HumansId': 2,
         "Mass": 0.1416 * mass,
         "CenterOfMass": [0, -0.4095*L['yfemurR'], 0],
         "GyrationRadius": array([0.329, 0.149, 0.329])*L['yfemurR']})
    bodies.append(
        {"HumansName": "ShankR",
         'HumansId': 3,
         "Mass": 0.0433 * mass,
         "CenterOfMass": [0, -0.4459*L['ytibiaR'], 0],
         "GyrationRadius": array([0.255, 0.103, 0.249])*L['ytibiaR']})
    bodies.append(
        {"HumansName": "FootR",
         'HumansId': 4,
         "Mass": 0.0137 * mass,
         "CenterOfMass": [
            0.4415*L['xfootR'] + T['Right foot heel'][0],
            -L['yfootR']/2.,
            0.],
         "GyrationRadius": array([0.124, 0.257, 0.245])*L['xfootR']})
    bodies.append(
        {"HumansName": "ThighL",
         'HumansId': 5,
         "Mass": 0.1416 * mass,
         "CenterOfMass": [0, -0.4095*L['yfemurL'], 0],
         "GyrationRadius": array([0.329, 0.149, 0.329])*L['yfemurL']})
    bodies.append(
        {"HumansName": "ShankL",
         'HumansId': 6,
         "Mass": 0.0433 * mass,
         "CenterOfMass": [0, -0.4459*L['ytibiaL'], 0],
         "GyrationRadius": array([0.255, 0.103, 0.249])*L['ytibiaL']})
    bodies.append(
        {"HumansName": "FootL",
         'HumansId': 7,
         "Mass": 0.0137 * mass,
         "CenterOfMass":
             [0.4415*L['xfootL'] + T['Left foot heel'][0],
             -L["yfootL"]/2,
             0.],
         "GyrationRadius": array([0.124, 0.257, 0.245])*L['xfootL']})
    bodies.append(
        {"HumansName": "UPT", # Upper Part of Trunk
         'HumansId': 8,
         "Mass": 0.1596 * mass,
         "CenterOfMass": [(L['xsternoclavR'] + L['xsternoclavL'])/4.,
                          0.7001*(L['ysternoclavR']+L['ysternoclavL'])/2.,
                          0.],
         "GyrationRadius": array([0.716, 0.659, 0.454])*L['ysternoclavR']})
    bodies.append(
        {"HumansName": "ScapulaR", # right shoulder
         'HumansId': 9,
         "Mass": 0.,
         "CenterOfMass": [0., 0., 0.],
         "GyrationRadius": array([0., 0., 0.])})
    bodies.append(
        {"HumansName": "ArmR",
         'HumansId': 10,
         "Mass": 0.0271 * mass,
         "CenterOfMass": [0., -0.5772*L['yhumerusR'], 0.],
         "GyrationRadius": array([0.285, 0.158, 0.269])*L['yhumerusR']})
    bodies.append(
        {"HumansName": "ForearmR",
         'HumansId': 11,
         "Mass": 0.0162 * mass,
         "CenterOfMass": [0., -0.4574*L['yforearmR'], 0.],
         "GyrationRadius": array([0.276, 0.121, 0.265])*L['yforearmR']})
    bodies.append(
        {"HumansName": "HandR",
         'HumansId': 12,
         "Mass": 0.0061 * mass,
         "CenterOfMass": [0, -0.3691*L['yhandR'], 0],
         "GyrationRadius": array([0.235, 0.184, 0.288])*L['yhandR']})
    bodies.append(
        {"HumansName": "ScapulaL", # left shoulder
         'HumansId': 13,
         "Mass": 0.,
         "CenterOfMass": [0., 0., 0.],
         "GyrationRadius": array([0., 0., 0.])})
    bodies.append(
        {"HumansName": "ArmL",
         'HumansId': 14,
         "Mass": 0.0271 * mass,
         "CenterOfMass": [0, -0.5772*L['yhumerusL'], 0.],
         "GyrationRadius": array([0.285, 0.158, 0.269])*L['yhumerusL']})
    bodies.append(
        {"HumansName": "ForearmL",
         'HumansId': 15,
         "Mass": 0.0162 * mass,
         "CenterOfMass": [0, -0.4574*L['yforearmL'], 0],
         "GyrationRadius": array([0.276, 0.121, 0.265])*L['yforearmL']})
    bodies.append(
        {"HumansName": "HandL",
         'HumansId': 16,
         "Mass": 0.0061 * mass,
         "CenterOfMass": [0, -0.3691*L['yhandL'], 0],
         "GyrationRadius": array([0.288, 0.184, 0.235])*L['yhandL']})
    bodies.append(
        {"HumansName": "Head",
         'HumansId': 17,
         "Mass": 0.0694 * mass,
         "CenterOfMass": [0, 0.4998*L['yhead'], 0],
         "GyrationRadius": array([0.303, 0.261, 0.315])*L['yhead']})
    return bodies

def _human36(world, height=1.741, mass=73, name=''):
    """

    TODO: HuMAnS' doc about inertia is erroneous (the real math is in the IOMatrix proc in DynamicData.maple)

    """
    assert isinstance(world, World)
    w = world
    L = anatomical_lengths(height)

    bodies = {}
    humansbodyid_to_humansbodyname_map = {}
    for b in _humans_bodies(height, mass):

        #mass matrix at com
        mass_g = b['Mass'] * diag(
            hstack((b['GyrationRadius']**2, (1,1,1))))
        H_fg = eye(4)
        H_fg[0:3,3] = b['CenterOfMass']
        H_gf = Hg.inv(H_fg)
        #mass matrix at body's frame origin:
        mass_o = dot(adjoint(H_gf).T, dot(mass_g, adjoint(H_gf)))

        bodies[b['HumansName']] = Body(
            name=b['HumansName'],
            mass=mass_o)

        humansbodyid_to_humansbodyname_map[b['HumansId']] = b['HumansName']

    rf = SubFrame(w.ground,
        Hg.transl(0, L['yfootL']+L['ytibiaL']+L['yfemurL'], 0))
    w.add_link(rf, FreeJoint(), bodies['LPT'])

    rf = SubFrame(bodies['LPT'], Hg.transl(0, 0, L['zhip']/2.))
    j = RzRyRxJoint()
    w.add_link(rf, j, bodies['ThighR'])

    rf = SubFrame(bodies['ThighR'], Hg.transl(0, -L['yfemurR'], 0))
    w.add_link(rf, RzJoint(), bodies['ShankR'])

    rf = SubFrame(bodies['ShankR'], Hg.transl(0, -L['ytibiaR'], 0))
    w.add_link(rf, RzRxJoint(), bodies['FootR'])

    rf = SubFrame(bodies['LPT'], Hg.transl(0, 0, -L['zhip']/2.))
    w.add_link(rf, RzRyRxJoint(), bodies['ThighL'])

    rf = SubFrame(bodies['ThighL'], Hg.transl(0, -L['yfemurL'], 0))
    w.add_link(rf, RzJoint(), bodies['ShankL'])

    rf = SubFrame(bodies['ShankL'], Hg.transl(0, -L['ytibiaL'], 0))
    w.add_link(rf, RzRxJoint(), bodies['FootL'])

    rf = SubFrame(bodies['LPT'], Hg.transl(-L['xvT10'], L['yvT10'], 0))
    w.add_link(rf, RzRyRxJoint(), bodies['UPT'])

    rf = SubFrame(bodies['UPT'],
        Hg.transl(L['xsternoclavR'], L['ysternoclavR'], L['zsternoclavR']))
    j = RyRxJoint()
    w.add_link(rf, j, bodies['ScapulaR'])

    rf = SubFrame(bodies['ScapulaR'],
        Hg.transl(-L['xshoulderR'], L['yshoulderR'], L['zshoulderR']))
    w.add_link(rf, RzRyRxJoint(), bodies['ArmR'])

    rf = SubFrame(bodies['ArmR'], Hg.transl(0, -L['yhumerusR'], 0))
    w.add_link(rf, RzRyJoint(), bodies['ForearmR'])

    rf = SubFrame(bodies['ForearmR'], Hg.transl(0, -L['yforearmR'], 0))
    w.add_link(rf, RzRxJoint(), bodies['HandR'])

    rf = SubFrame(bodies['UPT'], Hg.transl(
        L['xsternoclavL'], L['ysternoclavL'], -L['zsternoclavL']))
    w.add_link(rf, RyRxJoint(), bodies['ScapulaL'])

    rf = SubFrame(bodies['ScapulaL'],
        Hg.transl(-L['xshoulderL'], L['yshoulderL'], -L['zshoulderL']))
    w.add_link(rf, RzRyRxJoint(), bodies['ArmL'])

    rf = SubFrame(bodies['ArmL'], Hg.transl(0, -L['yhumerusL'], 0))
    w.add_link(rf, RzRyJoint(), bodies['ForearmL'])

    rf = SubFrame(bodies['ForearmL'], Hg.transl(0, -L['yforearmL'], 0))
    w.add_link(rf, RzRxJoint(), bodies['HandL'])

    rf = SubFrame(bodies['UPT'], Hg.transl(L['xvT10'], L['yvC7'], 0))
    w.add_link(rf, RzRyRxJoint(), bodies['Head'])

    # add tags
    tags = {}
    for t in _humans_tags(height):
        bodyname = humansbodyid_to_humansbodyname_map[t['HumansBodyId']]
        tag = SubFrame(
            bodies[bodyname],
            Hg.transl(t['Position'][0], t['Position'][1], t['Position'][2]),
            t['HumansName'])
        tags[t['HumansName']] = tag
        w.register(tag)

    # Add point shapes to the feet
    for k in ('Right foot toe tip', 'Right foot heel',
             'Right foot phalange 5', 'Right foot Phalange 1',
             'Left foot toe tip','Left foot heel',
              'Left foot phalange 5','Left foot phalange 1'):
        shape = Point(tags[k], name=k)
        w.register(shape)

    w.init()
    return (bodies, tags)

def add_human36(world, height=1.741, mass=73, name=''):
    _human36(height=height, mass=mass, name=name, world=world)


