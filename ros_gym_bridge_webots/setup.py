from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ros_gym_bridge_webots'],
    package_dir={'': 'src'})

setup(**setup_args)