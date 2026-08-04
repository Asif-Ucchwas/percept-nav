from setuptools import find_packages, setup
import glob

package_name = 'percept_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jannatul',
    maintainer_email='asifuzzamanucchwas@gmail.com',
    description='Real-time multi-sensor SLAM and dynamic obstacle navigation stack -- ROS2 Jazzy, Gazebo Harmonic, TurtleBot3',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_viewer_node = percept_nav.camera_viewer_node:main',
            'obstacle_detector_node = percept_nav.obstacle_detector_node:main',
            'sensor_fusion_node = percept_nav.sensor_fusion_node:main',
        ],
    },
)
