from setuptools import setup
import os

package_name = 'dg_description'


def get_all_files(directory):
    files = []
    for dirpath, _, filenames in os.walk(directory):
        for f in filenames:
            files.append(os.path.join(dirpath, f))
    return files


def recursive_data_files(source_root, target_root):

    data_files = []
    for dirpath, _, filenames in os.walk(source_root):
        if filenames:
            rel_path = os.path.relpath(dirpath, source_root)
            dest_dir = os.path.join(target_root, rel_path)
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((dest_dir, files))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), get_all_files('launch')),
    (os.path.join('share', package_name, 'urdf'), get_all_files('urdf')),
    (os.path.join('share', package_name, 'config'), get_all_files('config'))
]

data_files += recursive_data_files('meshes',
                                   os.path.join('share', package_name, 'meshes'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=data_files,
    zip_safe=True,
    maintainer='hongcheol',
    maintainer_email='khc@tesollo.com',
    description='The ' + package_name + ' package',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
