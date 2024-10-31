from setuptools import find_packages
from setuptools import setup

setup(
    name='pic4rl',
    version='0.0.0',
    packages=find_packages(
        include=('pic4rl', 'pic4rl.*')),
)
