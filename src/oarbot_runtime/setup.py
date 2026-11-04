from setuptools import find_packages, setup
setup(
    name='oarbot_runtime', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/oarbot_runtime']),
                ('share/oarbot_runtime', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='OARBot command watchdogs and human measurements', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'command_guard = oarbot_runtime.command_guard:main',
        'human_hand_measurements = oarbot_runtime.human_hand_measurements:main',
    ]},
)
