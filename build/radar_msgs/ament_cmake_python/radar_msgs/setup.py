from setuptools import find_packages
from setuptools import setup

setup(
    name='radar_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('radar_msgs', 'radar_msgs.*')),
)
