# coding = utf-8

"""
This module serve as a factory for anthropometric humanoid models.

The model is based on the ``human36`` anatomical model from the `HuMAnS
toolbox`_ software developed at the INRIA in Grenoble. 


TODO: report doc/code discrepancy to HuMAnS devs
TODO: decide if we support or no the ArborisName field in tags
TODO: fix errors in matlab-arboris

Finding the anatomical parameters in the HuMAnS toolbox source code
===================================================================

HuMAnS (at version 1.0.7) uses Maple-generated C-code for computing the
lagrangian model matrices and performs the integration and contact resolution in
scilab. More precisely, some maple code in the
``HuMAnS/LagrangianModel/Model/Human36/MapleCodeGeneration``  directory
describes

- the lagrangian dynamics,
- the contact kinematics,
- and the "tags" kinematics (in the HuMAnS vocable, Tags are caracteristic points such as the anatomical landmarks used for human motion capture).

These files generate the C file that are then called from scilab. The functions
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
from arboris import World, Body, SubFrame
import numpy as np
from numpy import array, diag, dot, hstack
import arboris.core.homogeneousmatrix as Hg
from arboris.core.homogeneousmatrix import adjoint
from arboris.core.joints import *

def anatomical_lengths(human_height):
    """

    >>> L = anatomical_lengths(1.741)
    >>> L['ysternoclavL']
    0.17061800000000002
    >>> L = anatomical_lengths(2.)
    >>> L['ysternoclavL']
    0.19600000000000001

    """
    lengths_list = _humans_anatomical_lengths(human_height)
    lengths_dict = {}
    for length in lengths_list:
        lengths_dict[length["HumansName"]] = length["Value"]
    return lengths_dict

def _humans_anatomical_lengths(human_height):
    """
    based on HuMAnS' ``LagrangianModel/Human36/Human36.c``

    Errors in matlab-arboris ?

    In this implementation, the "zshoulder", "zhip" and "xfoot" lengths are 
    computed as in HuMAnS, which is different from the way they were
    computed in matlab-arboris.

    Errors in HuMAnS ? 

    - 'xsternoclavR/L' (L(9) an L(19)) are documented as constant 
      (= 0,1817) but are implemented as size-dependant 
      (= 0.1052 *  human_height)
    - 'xvT10', 'xshoulderR/L' (L(2), L(12) an L(22)) are documented as 
      constant (= 0.09085) but are implemented as size-dependant 
      (= 0.0526 *  human_height)

    Examples:

    >>> def convert_to_array(lengths):
    ...     \"\"\"Convert the anatomical lengths dictionary into an array
    ...
    ...     The array is the same that the one returned by the
    ...     ``GetAnatomicalLengths`` function from the HuMAnS toolbox.
    ...     \"\"\"
    ...     L = np.zeros((len(lengths)))
    ...     for length in lengths:
    ...         L[length['HumansId']-1] = length["Value"]
    ...     return L
    >>> convert_to_array(_humans_anatomical_lengths(1.741))
    array([ 0.3612575,  0.0915766,  0.1744482,  0.0386502,  0.4340313,
            0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
            0.2127595,  0.0915766,  0.2816938,  0.0386502,  0.4340313,
            0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
            0.2127595,  0.0915766,  0.2816938,  0.2688104,  0.2688104,
            0.241999 ,  0.1899431,  0.1899431,  0.2428695,  0.2580162,
            0.2580162])
    >>> convert_to_array(_humans_anatomical_lengths(0.))
    array([ 0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
            0.0127,  0.    ,  0.    , -0.0127,  0.    ,  0.    ,  0.    ,
            0.    ,  0.    ,  0.    ,  0.0127,  0.    ,  0.    , -0.0127,
            0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
            0.    ,  0.    ,  0.    ])

    """

    yfoot =  0.0222 * human_height
    ytibia = 0.2493 * human_height
    yfemur = 0.2425 * human_height 
    ysternoclav = 0.0980 * human_height
    zsternoclav = 0.5 * 0.0254 # zsternoclav  = 0.5 inch 
    xsternoclav = 0.1052 *  human_height
    yshoulder = 0.0104 * human_height
    zshoulder = 0.1295 * human_height - zsternoclav
    xshoulder = 0.0526 *  human_height
    yhumerus = 0.1618 * human_height
    yforearm = 0.1544 * human_height
    yhand = 0.1091 * human_height	
    xfoot = 0.1482 * human_height

    lengths = []
    lengths.append({
        "HumansName": "yvT10", 
        'HumansId': 1,
        "Value": 0.2075*human_height,
        "Description": "hip joint centers/T10 vertebra"})
    lengths.append({
        "HumansName": "xvT10", 
        'HumansId': 2,
        "Value": 0.0526 *  human_height,
        "Description": "hip joint center middle point/T10 vertebra"})
    lengths.append({
        "HumansName": "zhip", 
        'HumansId': 3,
        "Value": 0.1002 * human_height,
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
        "Value": 0.139*human_height,
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
        "Value": 0.1395*human_height,
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
    """
    Examples:

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

def tags(human_height):
    """
    TODO: add body name
    """
    tdict = {}
    for t in _humans_tags(human_height):
        tdict[t['HumansName']] = {
            'Position': t['Position'],
        }
    return tdict

def _humans_tags(human_height):
    """
    - ``HumansName`` and ``HumansId`` come from 
      ``AdditionnalData.maple``. 
    - ``Position`` refer to ``AddL`` and come from ``Humans36.c``
    
    
    Examples:

    >>> def convert_to_array(tags):
    ...     \"\"\"Convert the tags dictionary into an array
    ...
    ...     The array elements are in the same order that in the array
    ...     returned by the ``GetTag2JointLengths`` function from the 
    ...     HuMAnS toolbox.
    ...     \"\"\"
    ...     t = np.zeros((len(tags),3))
    ...     for tag in tags:
    ...         t[tag['HumansId']-1,:] = tag["Position"][:]
    ...     return t
    >>> convert_to_array(_humans_tags(1.741))
    array([[ 0.2150135, -0.0386502,  0.       ],
           [-0.0431768, -0.0386502,  0.       ],
           [ 0.1152542, -0.0386502,  0.0531005],
           [ 0.1152542, -0.0386502, -0.0531005],
           [ 0.       , -0.4340313,  0.0433509],
           [ 0.       , -0.4221925,  0.050489 ],
           [ 0.       ,  0.       ,  0.076604 ],
           [ 0.0471811,  0.0637206,  0.1213477],
           [ 0.2150135, -0.0386502,  0.       ],
           [-0.0431768, -0.0386502,  0.       ],
           [ 0.1152542, -0.0386502, -0.0531005],
           [ 0.1152542, -0.0386502,  0.0531005],
           [ 0.       , -0.4340313, -0.0433509],
           [ 0.       , -0.4221925, -0.050489 ],
           [ 0.       ,  0.       , -0.076604 ],
           [ 0.0471811,  0.0637206, -0.1213477],
           [ 0.2122279,  0.       ,  0.       ],
           [ 0.1831532,  0.170618 ,  0.       ],
           [-0.0915766,  0.0525782,  0.2127595],
           [ 0.       , -0.2816938,  0.0367351],
           [ 0.       , -0.2668953,  0.0576271],
           [ 0.       , -0.1899431,  0.       ],
           [-0.0915766,  0.0525782, -0.2127595],
           [ 0.       , -0.2816938, -0.0367351],
           [ 0.       , -0.2668953, -0.0576271],
           [ 0.       , -0.1899431,  0.       ],
           [ 0.0915766,  0.241999 ,  0.       ],
           [ 0.       ,  0.2428695,  0.       ]])
    >>> convert_to_array(_humans_tags(0.))
    array([[ 0.    , -0.    ,  0.    ],
           [-0.    , -0.    ,  0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    , -0.    , -0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [ 0.    , -0.    ,  0.    ],
           [-0.    , -0.    ,  0.    ],
           [ 0.    , -0.    , -0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    , -0.    , -0.    ],
           [ 0.    , -0.    , -0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [ 0.    ,  0.    , -0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [-0.    ,  0.    , -0.0127],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    , -0.    ,  0.    ],
           [-0.    ,  0.    ,  0.0127],
           [ 0.    , -0.    , -0.    ],
           [ 0.    , -0.    , -0.    ],
           [ 0.    , -0.    ,  0.    ],
           [ 0.    ,  0.    ,  0.    ],
           [ 0.    ,  0.    ,  0.    ]])

    """
    h = human_height
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



def _humans_bodies(height, mass):
    """

    HuMAnS error ?

    - UPT center of mass depends on L["ysternoclavR"] but not on
    L["ysternoclavR"], we changed this behaviour


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
            0.4415*L['xfootR'] + T['Right foot heel']['Position'][0],
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
             [0.4415*L['xfootL'] + T['Left foot heel']['Position'][0],
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

def human36(height=1.741, mass=73, name='', world=None): 
    """

    TODO: HuMAnS' doc about inertia is erroneous (the real math is in the IOMatrix proc in DynamicData.maple)
    

    Test:

    >>> (w, bodies, tags) = human36()
    >>> w.update_geometric()
    >>> def tag_positions(tag_frames):
    ...     \"\"\"The tags absolute position
    ...
    ...     The array elements are in the same order that in the array
    ...     returned by the ``Tags`` function from the 
    ...     HuMAnS toolbox. In HuMAnS, though, there is an 29th line
    ...     which contains the center of mass position.
    ...     \"\"\"
    ...     from numpy import dot
    ...     pos_dict= {}
    ...     for (key, val) in tag_frames.iteritems():
    ...         pos_dict[key] = dot(val.body.pose, val._bpose)[0:3,3]
    ...
    ...     pos_array = np.zeros((len(tag_frames),3))
    ...     for t in _humans_tags(1.741):
    ...         pos_array[t['HumansId']-1,:] = pos_dict[t['HumansName']]
    ...     return pos_array
    >>> p = tag_positions(tags)
    >>> p
    array([[  2.15013500e-01,   2.08166817e-17,   8.72241000e-02],
           [ -4.31768000e-02,   2.08166817e-17,   8.72241000e-02],
           [  1.15254200e-01,   2.08166817e-17,   1.40324600e-01],
           [  1.15254200e-01,   2.08166817e-17,   3.41236000e-02],
           [  0.00000000e+00,   3.86502000e-02,   1.30575000e-01],
           [  0.00000000e+00,   4.72681500e-01,   1.37713100e-01],
           [  0.00000000e+00,   8.94874000e-01,   1.63828100e-01],
           [  4.71811000e-02,   9.58594600e-01,   1.21347700e-01],
           [  2.15013500e-01,   2.08166817e-17,  -8.72241000e-02],
           [ -4.31768000e-02,   2.08166817e-17,  -8.72241000e-02],
           [  1.15254200e-01,   2.08166817e-17,  -1.40324600e-01],
           [  1.15254200e-01,   2.08166817e-17,  -3.41236000e-02],
           [  0.00000000e+00,   3.86502000e-02,  -1.30575000e-01],
           [  0.00000000e+00,   4.72681500e-01,  -1.37713100e-01],
           [  0.00000000e+00,   8.94874000e-01,  -1.63828100e-01],
           [  4.71811000e-02,   9.58594600e-01,  -1.21347700e-01],
           [  1.20651300e-01,   1.25613150e+00,   0.00000000e+00],
           [  9.15766000e-02,   1.42674950e+00,   0.00000000e+00],
           [  0.00000000e+00,   1.47932770e+00,   2.25459500e-01],
           [  0.00000000e+00,   1.16316210e+00,   2.62194600e-01],
           [  0.00000000e+00,   8.96266800e-01,   2.83086600e-01],
           [  0.00000000e+00,   7.04408600e-01,   2.25459500e-01],
           [  0.00000000e+00,   1.47932770e+00,  -2.25459500e-01],
           [  0.00000000e+00,   1.16316210e+00,  -2.62194600e-01],
           [  0.00000000e+00,   8.96266800e-01,  -2.83086600e-01],
           [  0.00000000e+00,   7.04408600e-01,  -2.25459500e-01],
           [  0.00000000e+00,   1.49813050e+00,   0.00000000e+00],
           [  0.00000000e+00,   1.74100000e+00,   0.00000000e+00]])

Here is another HuMAnS result
 q  =
 
    0.         
    0.         
  - 0.8147779  
    0.         
    0.         
  - 0.0402589  
    0.         
    0.         
  - 0.2575124  
    0.         
    0.         
    0.2036690  
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.         
    0.2        
    0.         
    0.         
    0.         
    0.         
 
Tags(q)             
 ans  =
 
  - 0.2325857    0.2006062    0.0872241  
  - 0.4020078    0.3954354    0.0872241  
  - 0.2980468    0.2758841    0.1403246  
  - 0.2980468    0.2758841    0.0341236  
  - 0.3445103    0.3882163    0.130575   
  - 0.0169924    0.6730236    0.1377131  
    0.           1.094874     0.1638281  
    0.0471811    1.1585946    0.1213477  
    0.2746577    0.1978398  - 0.0872241  
    0.0168416    0.2117349  - 0.0872241  
    0.1750430    0.2032086  - 0.1403246  
    0.1750430    0.2032086  - 0.0341236  
    0.0620359    0.2480055  - 0.130575   
    0.0853943    0.6814078  - 0.1377131  
    0.           1.094874   - 0.1638281  
    0.0471811    1.1585946  - 0.1213477  
    0.1206513    1.4561315    0.         
    0.0915766    1.6267495    0.         
    0.           1.6793277    0.2254595  
    0.           1.3631621    0.2621946  
    0.           1.0962668    0.2830866  
    0.           0.9044086    0.2254595  
    0.           1.6793277  - 0.2254595  
    0.           1.3631621  - 0.2621946  
    0.           1.0962668  - 0.2830866  
    0.           0.9044086  - 0.2254595  
    0.           1.6981305    0.         
    0.           1.941        0.         
  - 0.0023268    1.1764976    1.894E-20  

           COM position as returned by HuMAnS for q(:)=0
           0.0019382    0.9712007    1.894E-20 
    > convert_to_array(_humans_tags(1.741))
    
    Test dynamical model (these were checked against HuMAnS):

    >>> w = human36()[0]
    >>> w.update_dynamic()
    >>> w.mass[5,5]
    73.000000000000014
    >>> w.mass[41, 41] # neck
    0.10430013572386694
    >>> w.mass[40,40] # neck
    0.020356790291165189
    >>> w.mass[39, 39] # neck
    0.10208399155688053
    >>> w.mass[16, 16] # foot
    0.001397215796713388
    >>> w.mass[17, 17] # foot
    0.0093741757009949949
    >>> w.mass[10, 10] # foot
    0.001397215796713388
    >>> w.mass[11, 11] # foot
    0.0093741757009949949



    """
    lengths = anatomical_lengths(height)
    L = lengths
    
    bodies = {}
    humansbodyid_to_humansbodyname_map = {}
    for b in _humans_bodies(height, mass):

        #mass matrix at com
        mass_g = b['Mass'] * diag(
            hstack((b['GyrationRadius']**2, (1,1,1))))
        H_gf = eye(4)
        H_gf[0:3,3] = b['CenterOfMass']
        
        #mass matrix at body's frame origin:
        mass_o = dot(adjoint(H_gf).T, dot(mass_g, adjoint(H_gf)))
        
        bodies[b['HumansName']] = Body(
            name=b['HumansName'],
            mass=mass_o)
        
        humansbodyid_to_humansbodyname_map[b['HumansId']] = b['HumansName']
    

    tags = {}
    for t in _humans_tags(height):
        bodyname = humansbodyid_to_humansbodyname_map[t['HumansBodyId']]
        tags[t['HumansName']] = SubFrame(
            bodies[bodyname],
            Hg.transl(t['Position'][0], t['Position'][1], t['Position'][2]),
            t['HumansName'])

    # Create a world
    if world is None:
        w = World()
    elif isinstance(world, World):
        w = world
    else:
        raise ValueError('the world argument must be an instance of the World class')

    rf = SubFrame(w.ground,
        Hg.transl(0, L['yfootL']+L['ytibiaL']+L['yfemurL'], 0))
    j = FreeJoint()
    j.attach(rf, bodies['LPT'])
    w.register(j)
    
    rf = SubFrame(bodies['LPT'], Hg.transl(0, 0, L['zhip']/2.))
    j = RzRyRxJoint()
    j.attach(rf, bodies['ThighR'])
    w.register(j)
    
    rf = SubFrame(bodies['ThighR'], Hg.transl(0, -L['yfemurR'], 0))
    j = RzJoint()
    j.attach(rf, bodies['ShankR'])
    w.register(j)

    
    rf = SubFrame(bodies['ShankR'], Hg.transl(0, -L['ytibiaR'], 0))
    j = RzRxJoint()
    j.attach(rf, bodies['FootR'])
    w.register(j)

    rf = SubFrame(bodies['LPT'], Hg.transl(0, 0, -L['zhip']/2.))
    j = RzRyRxJoint()
    j.attach(rf, bodies['ThighL'])
    w.register(j)

    rf = SubFrame(bodies['ThighL'], Hg.transl(0, -L['yfemurL'], 0))
    j = RzJoint()
    j.attach(rf,bodies['ShankL'])
    w.register(j)

    rf = SubFrame(bodies['ShankL'], Hg.transl(0, -L['ytibiaL'], 0))
    j = RzRxJoint()
    j.attach(rf, bodies['FootL'])
    w.register(j)

    rf = SubFrame(bodies['LPT'], Hg.transl(-L['xvT10'], L['yvT10'], 0))
    j = RzRyRxJoint()
    j.attach(rf, bodies['UPT'])
    w.register(j)
    
    rf = SubFrame(bodies['UPT'], 
        Hg.transl(L['xsternoclavR'], L['ysternoclavR'], L['zsternoclavR']))
    j = RyRxJoint()
    j.attach(rf, bodies['ScapulaR'])
    w.register(j)
    
    rf = SubFrame(bodies['ScapulaR'], 
        Hg.transl(-L['xshoulderR'], L['yshoulderR'], L['zshoulderR']))
    j = RzRyRxJoint()
    j.attach(rf, bodies['ArmR'])
    w.register(j)

    rf = SubFrame(bodies['ArmR'], Hg.transl(0, -L['yhumerusR'], 0))
    j = RzRyJoint()
    j.attach(rf, bodies['ForearmR'])
    w.register(j)
    
    rf = SubFrame(bodies['ForearmR'], Hg.transl(0, -L['yforearmR'], 0))
    w.register(RzRxJoint(frames=(rf, bodies['HandR'])))
    
    rf = SubFrame(bodies['UPT'], Hg.transl(
        L['xsternoclavL'], L['ysternoclavL'], -L['zsternoclavL']))
    w.register(RyRxJoint(frames=(rf, bodies['ScapulaL'])))
    
    rf = SubFrame(bodies['ScapulaL'], 
        Hg.transl(-L['xshoulderL'], L['yshoulderL'], -L['zshoulderL']))
    w.register(RzRyRxJoint(frames=(rf, bodies['ArmL'])))
    
    rf = SubFrame(bodies['ArmL'], Hg.transl(0, -L['yhumerusL'], 0))
    w.register(RzRyJoint(frames=(rf, bodies['ForearmL'])))
    
    rf = SubFrame(bodies['ForearmL'], Hg.transl(0, -L['yforearmL'], 0))
    w.register(RzRxJoint(frames=(rf, bodies['HandL'])))
    
    rf = SubFrame(bodies['UPT'], Hg.transl(L['xvT10'], L['yvC7'], 0))
    w.register(RzRyRxJoint(frames=(rf, bodies['Head'])))
   
    for t in tags.itervalues():
        w.register(t)
    w.initjointspace()
    return (w, bodies, tags)

