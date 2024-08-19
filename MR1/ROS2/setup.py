from setuptools import setup
import os
from glob import glob

package_name = 'farmer_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('farmer_launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenotic',
    maintainer_email='dinsokhengbds@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps4 = farmer_robot.Farmer_PS4:main',
            'can = farmer_robot.Farmer_CAN:main',
            'robot_red = farmer_robot.Farmer_PID_Red:main',
            'robot_blue = farmer_robot.Farmer_PID_Blue:main',
        ],
    },
)
