from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leap_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Package handling the streaming of data from the Leap cameras, as well as their fusion and visualization in RVIZ.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_streamer = leap_stream.leap_streamer:main',
            'fake_leap_streamer = leap_stream.fake_leap_streamer:main',
            'leap_visualizer = leap_stream.leap_visualizer:main',
            'leap_fusion = leap_stream.leap_fusion:main',
        ],
    },
)
