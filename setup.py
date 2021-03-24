from setuptools import setup
from glob import glob
package_name = 'vacuumcleaner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/launch.py')),
        ('share/' + package_name, glob('launch/rviz')),
        #('share/' + package_name, glob('models/robot.sdf')),
        ('share/' + package_name, glob('models/robot.xacro')),
        ('share/' + package_name, glob('models/worlds/square_2m'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='william@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_entity = vacuumcleaner.spawn_entity:main'
        ],
    },
)
