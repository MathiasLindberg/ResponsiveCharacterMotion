from setuptools import setup, find_packages

setup(name='character_motion',
      version='0.0.1',
      install_requires=['gym', 'pybullet', 'transformations', 'bvh'],
      packages=find_packages()
)