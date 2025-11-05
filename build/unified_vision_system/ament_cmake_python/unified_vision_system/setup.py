from setuptools import find_packages
from setuptools import setup

setup(
    name='unified_vision_system',
    version='1.0.0',
    packages=find_packages(
        include=('unified_vision_system', 'unified_vision_system.*')),
)
