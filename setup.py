from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'px4_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dronkab',
    maintainer_email='perrusquia832@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_manager = px4_driver.trajectory_manager:main',
            'px4_driver_node = px4_driver.px4_driver_node:main',
            'path_publisher = px4_driver.path_publisher:main'
        ],
    },
)
