## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

current_folder = os.path.dirname(os.path.realpath(__file__))
requirements_file_path = current_folder + '/requirements.txt'
required_libraries = []
if os.path.isfile(requirements_file_path):
    with open(requirements_file_path) as f:
        required_libraries = f.read().splitlines()

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['camera'],
    package_dir={'': 'include'},
    install_requires=required_libraries
)

setup(**setup_args)