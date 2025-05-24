from setuptools import find_packages
from setuptools import setup

setup(
    name='apf_control',
    version='0.0.0',
    packages=find_packages(
        include=('apf_control', 'apf_control.*')),
)
