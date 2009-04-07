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

The ``anatomicalLengths`` (aka. ``L``) parameters array computation::

  static double AnatMultSize[NLTAGS] = {
    0.2075, 0.0526, 0.1002, 0.0222, 0.2493,
    0.2425, 0.0980, 0, 0.1052, 0.0104, 
    0, 0.0526, 0.1618, 0.0222, 0.2493, 
    0.2425, 0.0980, 0, 0.1052, 0.0104, 
    0, 0.0526, 0.1618, 0.1544, 0.1544, 
    0.139, 0.1091, 0.1091, 0.1395, 0.1482, 
    0.1482};

  //New anatomical lengths
  for (i = 0; i < NLANAT; i++){
    anatomicalLengths[i] = AnatMultSize[i] * modelSize;
  }

  //Definition of the lengths which don't depends of the size
  anatomicalLengths[7] = 0.0127;
  anatomicalLengths[17] = 0.0127;
  anatomicalLengths[10] = 0.1295 * modelSize - anatomicalLengths[7];
  anatomicalLengths[20] = 0.1295 * modelSize - anatomicalLengths[17];

The ``Tag2JointLengths`` (aka. ``AddL``) parameters array computation::

  static double tagMultSize[NLTAGS] = {
    0.1235, 0, 0,
	-0.0248, 0, 0,
	0.0662, 0, 0.0305,
	0.0662, 0, -0.0305,
	0, 0, 0.0249,
	0, 0, 0.0290,
	0, 0, 0.0941,
	0.0271, 0.0366, 0.0697,
	0.1235, 0, 0,
	-0.0248, 0, 0,
	0.0662, 0, -0.0305,
	0.0662, 0, 0.0305,
	0, 0, -0.0249,
	0, 0, -0.0290,
	0, 0, -0.0941,
	0.0271, 0.0366, -0.0697,
	0.1219, 0, 0,
	0, 0, 0,
	0, 0.0198, 0,
	0, 0, 0.0211,
	0, -0.1533, 0.0331,
	0, -0.1091, 0,
	0, 0.0198, 0,
	0, 0, -0.0211,
	0, -0.1533, -0.0331,
	0, -0.1091, 0,
	-0.0392, 0, 0,
	0, 0.1395, 0};

  //New tags positions in their attached segment frames
  for (i = 0; i < NLTAGS; i++){
    tag2JointLengths[i] = tagMultSize[i] * modelSize;
  }

  tag2JointLengths[1] = -anatomicalLengths[3];
  tag2JointLengths[4] = -anatomicalLengths[3];
  tag2JointLengths[7] = -anatomicalLengths[3];
  tag2JointLengths[10] = -anatomicalLengths[3];
  tag2JointLengths[13] = -anatomicalLengths[4];
  tag2JointLengths[16] = -anatomicalLengths[5];
  tag2JointLengths[20] = tagMultSize[20] * modelSize - anatomicalLengths[2]/2;
  tag2JointLengths[25] = -anatomicalLengths[13];
  tag2JointLengths[28] = -anatomicalLengths[13];
  tag2JointLengths[31] = -anatomicalLengths[13];
  tag2JointLengths[34] = -anatomicalLengths[13];
  tag2JointLengths[37] = -anatomicalLengths[14];
  tag2JointLengths[40] = -anatomicalLengths[15];
  tag2JointLengths[44] = tagMultSize[44] * modelSize + anatomicalLengths[2]/2;
  tag2JointLengths[51] = (anatomicalLengths[8] + anatomicalLengths[18])/2;
  tag2JointLengths[52] = (anatomicalLengths[6] + anatomicalLengths[16])/2;
  tag2JointLengths[54] = -anatomicalLengths[11];
  tag2JointLengths[55] = tagMultSize[55] * modelSize + anatomicalLengths[9];
  tag2JointLengths[56] = anatomicalLengths[10];
  tag2JointLengths[58] = -anatomicalLengths[12];
  tag2JointLengths[66] = -anatomicalLengths[21];
  tag2JointLengths[67] =  tagMultSize[67] * modelSize + anatomicalLengths[19];
  tag2JointLengths[68] = -anatomicalLengths[20];
  tag2JointLengths[70] = -anatomicalLengths[22];
  tag2JointLengths[78] = anatomicalLengths[1];
  tag2JointLengths[79] = anatomicalLengths[25];


.. _`HuMAnS toolbox`:
    http://bipop.inrialpes.fr/software/humans/
    Human Motion Analysis and Simulation toolbox

"""

import arboris as arb
import numpy as np
import homogeneousmatrix as Hg

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
         "GyrationRadius": [0.2722, 0.2628, 0.226]})
    bodies.append(
        {"HumansName": "ThighR",
         'HumansId': 2,
         "Mass": 0.1416 * mass,
         "CenterOfMass": [0, -0.4095*L['yfemurR'], 0],
         "GyrationRadius": [0.329, 0.149, 0.329]})
    bodies.append(
        {"HumansName": "ShankR",
         'HumansId': 3,
         "Mass": 0.0433 * mass,
         "CenterOfMass": [0, -0.4459*L['ytibiaR'], 0],
         "GyrationRadius": [0.255, 0.103, 0.249]})
    bodies.append(
        {"HumansName": "FootR",
         'HumansId': 4,
         "Mass": 0.0137 * mass,
         "CenterOfMass": [
            0.4415*L['xfootR'] + T['Right foot heel']['Position'][0],
            -L['yfootR']/2.,
            0.],
         "GyrationRadius": [0.124, 0.257, 0.245]})
    bodies.append(
        {"HumansName": "ThighL",
         'HumansId': 5,
         "Mass": 0.1416 * mass,
         "CenterOfMass": [0, -0.4095*L['yfemurL'], 0],
         "GyrationRadius": [0.329, 0.149, 0.329]})
    bodies.append(
        {"HumansName": "ShankL",
         'HumansId': 6,
         "Mass": 0.0433 * mass,
         "CenterOfMass": [0, -0.4459*L['ytibiaL'], 0],
         "GyrationRadius": [0.255, 0.103, 0.249]})
    bodies.append(
        {"HumansName": "FootL",
         'HumansId': 7,
         "Mass": 0.0137 * mass,
         "CenterOfMass": 
             [0.4415*L['xfootL'] + T['Left foot heel']['Position'][0],
             -L["yfootL"]/2,
             0.],
         "GyrationRadius": [0.124, 0.257, 0.245]})
    bodies.append(
        {"HumansName": "UPT", # Upper Part of Trunk
         'HumansId': 8,
         "Mass": 0.1596 * mass,
         "CenterOfMass": [(L['xsternoclavR'] + L['xsternoclavL'])/4.,
                          0.7001*(L['ysternoclavR']+L['ysternoclavL'])/2.,
                          0.],
         "GyrationRadius": [0.716, 0.659, 0.454]})
    bodies.append(
        {"HumansName": "ScapulaR", # right shoulder
         'HumansId': 9,
         "Mass": 0.,
         "CenterOfMass": [0., 0., 0.],
         "GyrationRadius": [0., 0., 0.]})   
    bodies.append(
        {"HumansName": "ArmR",
         'HumansId': 10,
         "Mass": 0.0271 * mass,
         "CenterOfMass": [0., -0.5772*L['yhumerusR'], 0.],
         "GyrationRadius": [0.285, 0.158, 0.269]})   
    bodies.append(
        {"HumansName": "ForearmR",
         'HumansId': 11,
         "Mass": 0.0162 * mass,
         "CenterOfMass": [0., -0.4574*L['yforearmR'], 0.],
         "GyrationRadius": [0.276, 0.121, 0.265]})   
    bodies.append(
        {"HumansName": "HandR",
         'HumansId': 12,
         "Mass": 0.0061 * mass,
         "CenterOfMass": [0, -0.3691*L['yhandR'], 0],
         "GyrationRadius": [0.235, 0.184, 0.288]})   
    bodies.append(
        {"HumansName": "ScapulaL", # left shoulder
         'HumansId': 13,
         "Mass": 0.,
         "CenterOfMass": [0., 0., 0.],
         "GyrationRadius": [0., 0., 0.]})   
    bodies.append(
        {"HumansName": "ArmL",
         'HumansId': 14,
         "Mass": 0.0271 * mass,
         "CenterOfMass": [0, -0.5772*L['yhumerusL'], 0.],
         "GyrationRadius": [0.285, 0.158, 0.269]})   
    bodies.append(
        {"HumansName": "ForearmL",
         'HumansId': 15,
         "Mass": 0.0162 * mass,
         "CenterOfMass": [0, -0.4574*L['yforearmL'], 0],
         "GyrationRadius": [0.276, 0.121, 0.265]})   
    bodies.append(
        {"HumansName": "HandL",
         'HumansId': 16,
         "Mass": 0.0061 * mass,
         "CenterOfMass": [0, -0.3691*L['yhandL'], 0],
         "GyrationRadius": [0.288, 0.184, 0.235]})   
    bodies.append(
        {"HumansName": "Head",
         'HumansId': 17,
         "Mass": 0.0694 * mass,
         "CenterOfMass": [0, 0.4998*L['yhead'], 0],
         "GyrationRadius": [0.303, 0.261, 0.315]})
    return bodies


def human36(height=1.741, mass=73, name=''): 
    """

    >>> w = human36()
    """
    L = anatomical_lengths(height)
    bodies = {}
    for b in _humans_bodies(height, mass):
        bodies[b['HumansName']] = arb.Body(
        name = b['HumansName'])
        #'Mass'=b['Mass'],
        #'CenterOfMass': b['CenterOfMass'],
        #'GyrationRadius': b['GyrationRadius']}

    w = arb.World()


    rf = w.bodies[0].newframe(Hg.transl((0,0,L['yfootL']+L['ytibiaL']+L['yfemurL'])))
    #    n = bodies['LPT'].newframe(Hg.transl((0,0,-(L['yfootL']+L['ytibiaL']+L['yfemurL']))))
    w.addjoint(arb.FreeJoint(), rf, bodies['LPT'].frames[0])

    rf = bodies['LPT'].newframe(Hg.transl((0,0,L['zhip']/2.)))
    w.addjoint(arb.RzRyRxJoint(),rf,bodies['ThighR'].frames[0])
    
    f = bodies['ThighR'].newframe(Hg.transl((0,-L['yfemurR'],0)))
    w.addjoint(arb.HingeJoint(),f,bodies['ShankR'].frames[0])
    return w

if __name__ == "__main__":
    import doctest
    doctest.testmod()

