

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['reverse_server'],
    scripts=['scripts/reverser.py'],
)

setup(**d)
