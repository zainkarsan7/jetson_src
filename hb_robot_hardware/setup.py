from setuptools import setup
from glob import glob
package_name = 'hb_robot_hardware'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='HB Robot',
      maintainer_email='maintainer@example.invalid', description='MPU6050 and camera flap serial bridge',
      license='Apache-2.0', entry_points={'console_scripts': ['nano_bridge = hb_robot_hardware.nano_bridge:main']})
