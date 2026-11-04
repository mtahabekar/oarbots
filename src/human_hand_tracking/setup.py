from setuptools import find_packages, setup

setup(
    name='human_hand_tracking', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/human_hand_tracking']),
                ('share/human_hand_tracking', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='OARBot maintainers', maintainer_email='oconna4@rpi.edu',
    description='Tagged glove tracking', license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['fusion = human_hand_tracking.node:main']},
)
