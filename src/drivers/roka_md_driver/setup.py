import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'roka_md_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryoo',
    maintainer_email='ryoo@example.com',
    description='Guarded MD200T driver for four MDH250 wheels.',
    license='MIT',
    entry_points={'console_scripts': [
        'roka_md_driver_node = roka_md_driver.md_driver_node:main',
        'roka_xbox_teleop = roka_md_driver.xbox_teleop_node:main',
    ]},
)
