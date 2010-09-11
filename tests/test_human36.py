
import unittest
from arboristest import TestCase
from arboris.all import *
from arboris.robots.human36 import anat_lengths_from_height
import h5py

write = False

# List of the anatomical lengths defined in HuMAnS, it the proper order.
HuMAnS_lengths = (
        "yvT10", # HuMAnS id: 1
        "xvT10", # HuMAnS id: 2
        "zhip", # HuMAnS id: 3
        "yfootR", # HuMAnS id: 4
        "ytibiaR", # HuMAnS id: 5
        "yfemurR", # HuMAnS id: 6
        "ysternoclavR", # HuMAnS id: 7
        "zsternoclavR", # HuMAnS id: 8
        "xsternoclavR", # HuMAnS id: 9
        "yshoulderR", # HuMAnS id: 10
        "zshoulderR", # HuMAnS id: 11
        "xshoulderR", # HuMAnS id: 12
        "yhumerusR", # HuMAnS id: 13
        "yfootL", # HuMAnS id: 14
        "ytibiaL", # HuMAnS id: 15
        "yfemurL", # HuMAnS id: 16
        "ysternoclavL", # HuMAnS id: 17
        "zsternoclavL", # HuMAnS id: 18
        "xsternoclavL", # HuMAnS id: 19
        "yshoulderL", # HuMAnS id: 20
        "zshoulderL", # HuMAnS id: 21
        "xshoulderL", # HuMAnS id: 22
        "yhumerusL", # HuMAnS id: 23
        "yforearmR", # HuMAnS id: 24
        "yforearmL", # HuMAnS id: 25
        "yvC7", # HuMAnS id: 26
        "yhandR", # HuMAnS id: 27
        "yhandL", # HuMAnS id: 28
        "yhead", # HuMAnS id: 29
        "xfootR", # HuMAnS id: 30
        "xfootL") # HuMAnS id: 31
# List of the tags (sites/feature points) defined in HuMAnS, it the proper
# order.
HuMAnS_tags = (
        'Right foot toe tip', # HuMAnS id: 1
        'Right foot heel', # HuMAnS id: 2
        'Right foot phalange 5', # HuMAnS id: 3
        'Right foot Phalange 1', # HuMAnS id: 4
        'Right foot lateral malleolus', # HuMAnS id: 5
        'Femoral lateral epicondyle', # HuMAnS id: 6
        'Right great trochanter', # HuMAnS id: 7
        'Right iliac crest', # HuMAnS id: 8
        'Left foot toe tip', # HuMAnS id: 9
        'Left foot heel', # HuMAnS id: 10
        'Left foot phalange 5', # HuMAnS id: 11
        'Left foot phalange 1', # HuMAnS id: 12
        'Left foot lateral malleolus', # HuMAnS id: 13
        'Left femoral lateral epicondyle', # HuMAnS id: 14
        'Left great trochanter', # HuMAnS id: 15
        'Left iliac crest', # HuMAnS id: 16
        'Substernale (Xyphoid)', # HuMAnS id: 17
        'Suprasternale', # HuMAnS id: 18
        'Right acromion', # HuMAnS id: 19
        'Right humeral lateral epicondyle (radiale)', # HuMAnS id: 20
        'Right stylion', # HuMAnS id: 21
        'Right 3rd dactylion', # HuMAnS id: 22
        'Left acromion', # HuMAnS id: 23
        'Left humeral lateral epicondyle (radiale)', # HuMAnS id: 24
        'Left stylion', # HuMAnS id: 25
        'Left 3rd dactylion', # HuMAnS id: 26
        'Cervicale', # HuMAnS id: 27
        'Vertex') # HuMAnS id: 28
HuMAnS_bodies = (
         "LPT", # HuMAnS id: 1
         "ThighR", # HuMAnS id: 2
         "ShankR", # HuMAnS id: 3
         "FootR", # HuMAnS id: 4
         "ThighL", # HuMAnS id: 5
         "ShankL", # HuMAnS id: 6
         "FootL", # HuMAnS id: 7
         "UPT", # HuMAnS id: 8
         "ScapulaR", # HuMAnS id: 9
         "ArmR", # HuMAnS id: 10
         "ForearmR", # HuMAnS id: 11
         "HandR", # HuMAnS id: 12
         "ScapulaL", # HuMAnS id: 13
         "ArmL", # HuMAnS id: 14
         "ForearmL", # HuMAnS id: 15
         "HandL", # HuMAnS id: 16
         "Head") # HuMAnS id: 17

class Human36Masses(TestCase):

    def setUp(self):
        self.world = World()
        add_human36(self.world)
        if write:
            mode = 'a'
        else:
            mode = 'r'
        self.file = h5py.File('tests/human36.h5', mode)

    def test_masses_against_h5(self):
        group = self.file.require_group('masses')
        for b in self.world.iterbodies():
            #print(b.name)
            if write:
                d = group.require_dataset(b.name, (6,6), 'f8')
                d[:,:] = b.mass
            else:
                self.assertListsAlmostEqual(group[b.name], b.mass)

    def tearDown(self):
        self.file.close()

class Human36AgainstHumansTestCase(TestCase):
    """Test the human36 robot generator."""

    def test_anatlengths_against_HuMAnS(self):
        """Check anatomical lengths are the same than in HuMAnS.
        """
        # anatomical lengths taken from HuMAnS
        hld = {1.741:
                (0.3612575,  0.0915766,  0.1744482,  0.0386502,  0.4340313,
                 0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
                 0.2127595,  0.0915766,  0.2816938,  0.0386502,  0.4340313,
                 0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
                 0.2127595,  0.0915766,  0.2816938,  0.2688104,  0.2688104,
                 0.241999 ,  0.1899431,  0.1899431,  0.2428695,  0.2580162,
                 0.2580162),
                  0.:
                (0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
                 0.0127,  0.    ,  0.    , -0.0127,  0.    ,  0.    ,  0.    ,
                 0.    ,  0.    ,  0.    ,  0.0127,  0.    ,  0.    , -0.0127,
                 0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
                 0.    ,  0.    ,  0.    )}
        for h, hl in hld.iteritems():
            lengths = anat_lengths_from_height(h)
            al = [lengths[n] for n in HuMAnS_lengths]
            self.assertListsAlmostEqual(hl, al)

    def test_tags_local_position_against_HuMAnS(self):
        htd = {1.741:
                (( 0.2150135, -0.0386502,  0.       ),
                 (-0.0431768, -0.0386502,  0.       ),
                 ( 0.1152542, -0.0386502,  0.0531005),
                 ( 0.1152542, -0.0386502, -0.0531005),
                 ( 0.       , -0.4340313,  0.0433509),
                 ( 0.       , -0.4221925,  0.050489 ),
                 ( 0.       ,  0.       ,  0.076604 ),
                 ( 0.0471811,  0.0637206,  0.1213477),
                 ( 0.2150135, -0.0386502,  0.       ),
                 (-0.0431768, -0.0386502,  0.       ),
                 ( 0.1152542, -0.0386502, -0.0531005),
                 ( 0.1152542, -0.0386502,  0.0531005),
                 ( 0.       , -0.4340313, -0.0433509),
                 ( 0.       , -0.4221925, -0.050489 ),
                 ( 0.       ,  0.       , -0.076604 ),
                 ( 0.0471811,  0.0637206, -0.1213477),
                 ( 0.2122279,  0.       ,  0.       ),
                 ( 0.1831532,  0.170618 ,  0.       ),
                 (-0.0915766,  0.0525782,  0.2127595),
                 ( 0.       , -0.2816938,  0.0367351),
                 ( 0.       , -0.2668953,  0.0576271),
                 ( 0.       , -0.1899431,  0.       ),
                 (-0.0915766,  0.0525782, -0.2127595),
                 ( 0.       , -0.2816938, -0.0367351),
                 ( 0.       , -0.2668953, -0.0576271),
                 ( 0.       , -0.1899431,  0.       ),
                 ( 0.0915766,  0.241999 ,  0.       ),
                 ( 0.       ,  0.2428695,  0.       )),
              0.:
                (( 0.    , -0.    ,  0.    ),
                 (-0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 (-0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 ( 0.    ,  0.    , -0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 (-0.    ,  0.    , -0.0127),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 (-0.    ,  0.    ,  0.0127),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    , -0.    , -0.    ),
                 ( 0.    , -0.    ,  0.    ),
                 ( 0.    ,  0.    ,  0.    ),
                 ( 0.    ,  0.    ,  0.    ))}
        def tags_local_positions(tag_frames):
            """returns the tags local positions

            The array elements are in the same order that in the array
            returned by the ``Tags`` function from the
            HuMAnS toolbox. In HuMAnS, though, there is an 29th line
            which contains the center of mass position.
            """
            from numpy import dot
            positions = []
            for name in HuMAnS_tags:
                frame = tag_frames[name]
                positions.append(frame._bpose[0:3,3])
            return positions
        for h, ht in htd.iteritems():
            (joints, bodies, tags, shapes) = add_human36(World(), height=h,
                                                         return_lists=True)
            at = tags_local_positions(tags)
            self.assertListsAlmostEqual(ht, at)

    def test_tags_absolute_positions_against_HuMAnS(self):
        # tags absolute positions as returned from HuMAnS for q(:)==0
        # the last line, representing the COM position was stripped.
        # it was 0.0019382    0.9712007    1.894E-20
        htd = {'q0':
                ((  2.15013500e-01,   2.08166817e-17,   8.72241000e-02),
                 ( -4.31768000e-02,   2.08166817e-17,   8.72241000e-02),
                 (  1.15254200e-01,   2.08166817e-17,   1.40324600e-01),
                 (  1.15254200e-01,   2.08166817e-17,   3.41236000e-02),
                 (  0.00000000e+00,   3.86502000e-02,   1.30575000e-01),
                 (  0.00000000e+00,   4.72681500e-01,   1.37713100e-01),
                 (  0.00000000e+00,   8.94874000e-01,   1.63828100e-01),
                 (  4.71811000e-02,   9.58594600e-01,   1.21347700e-01),
                 (  2.15013500e-01,   2.08166817e-17,  -8.72241000e-02),
                 ( -4.31768000e-02,   2.08166817e-17,  -8.72241000e-02),
                 (  1.15254200e-01,   2.08166817e-17,  -1.40324600e-01),
                 (  1.15254200e-01,   2.08166817e-17,  -3.41236000e-02),
                 (  0.00000000e+00,   3.86502000e-02,  -1.30575000e-01),
                 (  0.00000000e+00,   4.72681500e-01,  -1.37713100e-01),
                 (  0.00000000e+00,   8.94874000e-01,  -1.63828100e-01),
                 (  4.71811000e-02,   9.58594600e-01,  -1.21347700e-01),
                 (  1.20651300e-01,   1.25613150e+00,   0.00000000e+00),
                 (  9.15766000e-02,   1.42674950e+00,   0.00000000e+00),
                 (  0.00000000e+00,   1.47932770e+00,   2.25459500e-01),
                 (  0.00000000e+00,   1.16316210e+00,   2.62194600e-01),
                 (  0.00000000e+00,   8.96266800e-01,   2.83086600e-01),
                 (  0.00000000e+00,   7.04408600e-01,   2.25459500e-01),
                 (  0.00000000e+00,   1.47932770e+00,  -2.25459500e-01),
                 (  0.00000000e+00,   1.16316210e+00,  -2.62194600e-01),
                 (  0.00000000e+00,   8.96266800e-01,  -2.83086600e-01),
                 (  0.00000000e+00,   7.04408600e-01,  -2.25459500e-01),
                 (  0.00000000e+00,   1.49813050e+00,   0.00000000e+00),
                 (  0.00000000e+00,   1.74100000e+00,   0.00000000e+00))}
        w = World()
        (joints, bodies, tags, shapes) = add_human36(w, return_lists=True)
        w.update_geometric()
        def tags_absolute_positions(tag_frames):
            """returns the tags absolute positions

            The array elements are in the same order that in the array
            returned by the ``Tags`` function from the
            HuMAnS toolbox. In HuMAnS, though, there is an 29th line
            which contains the center of mass position.
            """
            from numpy import dot
            positions = []
            for name in HuMAnS_tags:
                frame = tag_frames[name]
                positions.append(dot(frame.body.pose, frame._bpose)[0:3,3])
            return positions
        p = tags_absolute_positions(tags)
        self.assertListsAlmostEqual(p, htd['q0'])

if __name__ == '__main__':
    unittest.main()
