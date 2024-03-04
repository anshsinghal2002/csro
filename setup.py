# From https://edgenoon-ai.github.io/robot-operating-system/create-own-ros-service-python/
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['project_name'],
    package_dir={'': 'src'}
)

setup(**setup_args)

