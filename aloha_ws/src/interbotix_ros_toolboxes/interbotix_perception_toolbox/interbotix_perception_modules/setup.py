from glob import glob
import os

from setuptools import (
    find_packages,
    setup,
)

package_name = 'interbotix_perception_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'ui'),
            glob('src/ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author_email='trsupport@trossenrobotics.com',
    author='Solomon Wiznitzer',
    maintainer='Luke Schmitt',
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_perception_modules package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'armtag_tuner_gui = interbotix_perception_modules.armtag_tuner_gui:main',
            'picture_snapper = interbotix_perception_modules.picture_snapper:main',
            'pointcloud_tuner_gui = interbotix_perception_modules.pointcloud_tuner_gui:main',
        ],
    },
)
