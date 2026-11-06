from setuptools import find_packages, setup

setup(name='fabric_reference_reconstruction', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fabric_reference_reconstruction']),
                ('share/fabric_reference_reconstruction', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True, maintainer='OARBot maintainers',
    maintainer_email='oconna4@rpi.edu', description='Offline fabric reconstruction', license='Apache-2.0',
    tests_require=['pytest'], entry_points={'console_scripts': [
        'reconstruct = fabric_reference_reconstruction.cli:main',
        'import_bag = fabric_reference_reconstruction.bag:main',
        'synthetic = fabric_reference_reconstruction.synthetic:main']})
