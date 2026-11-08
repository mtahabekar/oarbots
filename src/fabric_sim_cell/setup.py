from glob import glob

from setuptools import find_packages, setup

setup(
    name='fabric_sim_cell', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_sim_cell']),
                ('share/fabric_sim_cell', ['package.xml', 'README.md']),
                ('share/fabric_sim_cell/config', glob('config/*')),
                ('share/fabric_sim_cell/launch', glob('launch/*.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='Fabric estimation against the simulated OARBot cell', license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'calibration_publisher = fabric_sim_cell.nodes:calibration_main',
        'oracle_hands = fabric_sim_cell.nodes:oracle_hands_main',
        'recorder = fabric_sim_cell.nodes:recorder_main',
        'grasp_gate = fabric_sim_cell.nodes:grasp_gate_main',
        'scorecard = fabric_sim_cell.scorecard:main',
        'demo_markers = fabric_sim_cell.nodes:demo_markers_main',
        'evaluate = fabric_sim_cell.evaluate:main']},
)
