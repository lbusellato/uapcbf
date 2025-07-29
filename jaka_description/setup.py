from setuptools import find_packages, setup
from glob import glob

package_name = 'jaka_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/meshes/jaka_zu5_meshes', glob('meshes/jaka_zu5_meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo Busellato',
    maintainer_email='lorenzo.busellato@gmail.com',
    description='This package the description and moveit configuration of the JAKA ZU 5 robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_visualizer_node = jaka_description.leap_visualizer_node:main',
            'log_playback_node = jaka_description.log_playback_node:main'
        ],
    },
)
