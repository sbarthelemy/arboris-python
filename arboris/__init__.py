# coding=utf-8
"""Check which modules are availlable."""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

def optional_modules():
    """A list of optional modules which are available."""
    mods = []
    try:
        import visu_osg
        mods.append('visu_osg')
    except ImportError:
        pass
    try:
        import qpcontroller
        mods.append('qpcontroller')
    except ImportError:
        pass
    return mods

__all__ = ['adjointmatrix', 'collisions', 'constraints', 'controllers',
           'core', 'homogeneousmatrix', 'joints', 'massmatrix',
           'observers', 'rigidmotion', 'robots', 'shapes', 'twistvector']
__all__.extend(optional_modules())
