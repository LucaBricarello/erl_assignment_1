from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'erl_assignment_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Installa i file di launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Installa i file URDF e XACRO
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # Installa i mondi Gazebo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
        # Installa le configurazioni Rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        
        # Installa config
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # meshes:
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='luca.bricarello@outlook.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = erl_assignment_1.mission:main',
        ],
    },
)
