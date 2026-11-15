from glob import glob

from setuptools import setup

setup(
    name='fabric_cell', version='0.1.0', packages=[],
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_cell']),
                ('share/fabric_cell', ['package.xml', 'README.md']),
                ('share/fabric_cell/config', glob('config/*')),
                ('share/fabric_cell/launch', glob('launch/*.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='Fabric estimator, twin and CBF on the real OARBot cell', license='Apache-2.0', tests_require=['pytest'],
)
