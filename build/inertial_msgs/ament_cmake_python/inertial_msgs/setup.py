from setuptools import find_packages
from setuptools import setup

setup(
    name='inertial_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('inertial_msgs', 'inertial_msgs.*')),
)
