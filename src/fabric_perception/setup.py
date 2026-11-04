from glob import glob
from setuptools import find_packages, setup

setup(
    name='fabric_perception', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_perception']),
                ('share/fabric_perception', ['package.xml']),
                ('share/fabric_perception/config', glob('config/*.yaml')),
                ('share/fabric_perception/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='Per-camera fabric observations', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['camera = fabric_perception.node:main']},
)
