from setuptools import setup

package_name = 'gabiru_track'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos@usp.br',
    description='Nodo para cargar y segmentar la pista para el proyecto Gabiru',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'segmentation_node = gabiru_track.segmentation_node:main',
        ],
    },
)
