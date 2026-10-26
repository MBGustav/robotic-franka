from glob import glob

from setuptools import setup

package_name = 'aided_building_gripper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aided Building Team',
    maintainer_email='team@example.com',
    description='Franka Hand control: same action interface in Gazebo and on hardware',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_gripper_server = aided_building_gripper.sim_gripper_server:main',
            'gripper_cli = aided_building_gripper.gripper_cli:main',
        ],
    },
)
