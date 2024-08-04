from setuptools import setup
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
        ('lib/python3.10/site-packages/python_package_include', glob(f'{package_name}/python_package_include/*.py')),
        ('lib/python3.10/site-packages/dynamixel_sdk',
         glob(f'{package_name}/python_package_include/dynamixel_sdk/*')),
        ('lib/python3.8/site-packages/python_package_include', glob(f'{package_name}/python_package_include/*.py')),
        ('lib/python3.8/site-packages/dynamixel_sdk',
         glob(f'{package_name}/python_package_include/dynamixel_sdk/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elian_NEPPEL',
    maintainer_email='example@dc.tohoku.ac.jp',
    description='interfaces with the motors',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            f'u2d2_dyna_controller = {package_name}.u2d2_dyna_controller:main',
            f'angle_remapper = {package_name}.angle_remapper:main',

        ],
    },
)
