## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
# BE CAREFUL! IT MAY NEED CHANGES!!
setup_args = generate_distutils_setup(
    packages=['image_processing_package'],
    package_dir={'': 'src'})

setup(**setup_args)