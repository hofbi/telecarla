# pylint: skip-file

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["telecarla_manual_control"], package_dir={"": "src"}
)

setup(**d)
