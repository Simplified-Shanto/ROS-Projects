from setuptools import setup
import os, glob

package_name = 'wit_ros2_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wit_ros2_imu = wit_ros2_imu.wit_ros2_imu:main',
            'imu_tf_broadcaster = wit_ros2_imu.imu_tf_broadcaster:main',
        ],
    },
)
