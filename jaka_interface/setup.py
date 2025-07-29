from setuptools import find_packages, setup

package_name = 'jaka_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaka',
    maintainer_email='jaka@todo.com',
    description='This package implements the interface with the JAKA ZU 5 robot, wrapping its Python SDK functions in a ROS2 node.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface = jaka_interface.interface:main'
        ],
    },
)
