from glob import glob

from setuptools import find_packages, setup

setup(
    name='fabric_twin', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_twin']),
                ('share/fabric_twin', ['package.xml', 'README.md']),
                ('share/fabric_twin/config', glob('config/*')),
                ('share/fabric_twin/launch', glob('launch/*.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='Simulated fabric twin slaved to measured grips', license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'twin = fabric_twin.node:main',
        'parameter_bank = fabric_twin.bank_node:main',
        'twin_recorder = fabric_twin.recorder:main']},
)
