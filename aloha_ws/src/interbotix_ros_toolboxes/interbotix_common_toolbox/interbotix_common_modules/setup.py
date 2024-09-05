from setuptools import (
    find_packages,
    setup,
)

package_name = 'interbotix_common_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author_email='trsupport@trossenrobotics.com',
    author='Solomon Wiznitzer',
    maintainer='Luke Schmitt',
    maintainer_email='luke@trossenrobotics.com',
    description='The interbotix_common_modules package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
