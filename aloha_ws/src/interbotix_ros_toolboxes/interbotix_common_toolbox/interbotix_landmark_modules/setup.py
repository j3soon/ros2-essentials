from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)

package_name = 'interbotix_landmark_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'landmarks'),
            glob('landmarks/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author_email='trsupport@trossenrobotics.com',
    author='Luke Schmitt',
    maintainer='Luke Schmitt',
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_landmark_modules package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_finder = interbotix_landmark_modules.landmark_finder:main',
            'landmark_manager = interbotix_landmark_modules.landmark_manager:main',
            'tf_map_to_landmark = interbotix_landmark_modules.tf_map_to_landmark:main',
        ],
    },
)
