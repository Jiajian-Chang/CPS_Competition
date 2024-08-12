from setuptools import setup
import glob, os

package_name = 'yolo_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.module'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['yolo_ros2/config/coco.yaml','yolo_ros2/config/coco.names']),
        (os.path.join('share', package_name, 'weights'), ['yolo_ros2/weights/weight_AP05:0.253207_280-epoch.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='jiajianchang1994@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_ros2.yolo_detector_node:main',
            'lidar_filter = yolo_ros2.lidar_filter:main',
        ],
    },
)
