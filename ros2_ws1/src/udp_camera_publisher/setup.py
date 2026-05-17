from setuptools import setup
import os
import glob

package_name = 'udp_camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shanto',
    maintainer_email='shanto@todo.todo',
    description='Publish a UDP H.264 camera stream as ROS 2 image messages.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_camera_publisher = udp_camera_publisher.udp_camera_publisher_node:main',
        ],
    },
)
