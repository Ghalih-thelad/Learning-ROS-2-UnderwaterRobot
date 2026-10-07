import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

        ('share/' + package_name,
        ['package.xml']),

        ('share/' + package_name + '/launch',
        glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'meshes'),
        glob('meshes/*')),

        ('share/' + package_name + '/config',
        glob('config/*.yaml')),

        ('share/' + package_name + '/urdf',
        glob('urdf/*')),

        ('share/' + package_name + '/worlds',
        glob('worlds/*')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghalih-a-salamun',
    maintainer_email='ghalih-a-salamun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fin_controller = auv_sim.fin_controller:main',
            'thruster_controller = auv_sim.thruster_controller:main',
            'gamepad_controller = auv_sim.gamepad_controller:main',
        ],
    },
)
