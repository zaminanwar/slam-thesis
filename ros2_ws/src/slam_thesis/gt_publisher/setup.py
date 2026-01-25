from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gt_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Ground truth pose publisher from Gazebo model state',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gt_publisher = gt_publisher.gt_publisher_node:main',
        ],
    },
)
