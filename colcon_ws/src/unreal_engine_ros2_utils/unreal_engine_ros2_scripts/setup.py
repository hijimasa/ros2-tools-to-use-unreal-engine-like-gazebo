import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'unreal_engine_ros2_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('blender_scripts/*.py')),
        (os.path.join('share',package_name), glob('unreal_engine_scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launcher = unreal_engine_ros2_scripts.launcher:main',
            'spawn_robot = unreal_engine_ros2_scripts.spawn_robot:main',
            'add_usd = unreal_engine_ros2_scripts.add_usd:main',
        ],
    },
)
