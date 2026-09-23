from glob import glob
from setuptools import find_packages, setup

name = 'handeye_calibration_ros2'
setup(
    name=name, version='0.1.1', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'LICENSE', 'NOTICE.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Local workspace maintainer', maintainer_email='maintainer@example.com',
    description='Standalone ROS 2 hand-eye calibration', license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'calibration_node = handeye_calibration_ros2.node:main',
        'calibration_gui = handeye_calibration_ros2.gui:main',
        'handeye = handeye_calibration_ros2.cli:main',
    ]},
)
