from glob import glob
from setuptools import find_packages, setup

setup(name='fabric_state_estimator', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_state_estimator']),
                ('share/fabric_state_estimator', ['package.xml']),
                ('share/fabric_state_estimator/config', glob('config/*.yaml')+glob('config/*.txt')),
                ('share/fabric_state_estimator/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='OARBot maintainers',
    maintainer_email='oconna4@rpi.edu', description='Causal ensemble fabric observer', license='Apache-2.0',
    tests_require=['pytest'], entry_points={'console_scripts': [
        'observer = fabric_state_estimator.node:main',
        'evaluate = fabric_state_estimator.metrics:main',
        'visualize = fabric_state_estimator.visualization:main',
        'replay = fabric_state_estimator.replay:main']})
