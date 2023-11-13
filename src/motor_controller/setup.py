from setuptools import setup
from setuptools import find_packages
# This is to import params from launchfiles
import os
from glob import glob
package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.8/site-packages/python_package_include', glob(f'{package_name}/python_package_include/*.py')),
        ('lib/python3.8/site-packages/dynamixel_sdk',
         glob(f'{package_name}/python_package_include/dynamixel_sdk/*')),
        ('share/' + package_name, glob('urdf/*')),
        ('share/' + package_name, glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elian_NEPPEL',
    maintainer_email='neppel.elian.s6@dc.tohoku.ac.jp',
    description='interfaces with the motors',
    license='Apache License 2.0',
    # set the shortcuts to run an executable.py, more specifically function of it
    entry_points={
        'console_scripts': [
            f'multi_dynamixel = {package_name}.multi_dynamixel:main',
            f'moonbot_interface = {package_name}.interface_for_moonbot:main',

        ],
    },
)
