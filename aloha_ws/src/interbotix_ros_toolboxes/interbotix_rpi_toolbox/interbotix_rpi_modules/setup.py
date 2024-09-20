from setuptools import (
    setup,
)

package_name = 'interbotix_rpi_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
    maintainer_email='trsupport@trossenrobotics.com',
    description='The interbotix_rpi_modules package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
