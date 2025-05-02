from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dd_robot'

def package_files(directory):
    # Recursively collect all files under directory
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [filepath]))
    return paths

model_files = package_files('models/TurtlebotArena')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),

        (os.path.join('share',package_name,'config'),
         glob(os.path.join('config','*.yaml'))),

        (os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.urdf')) + glob(os.path.join('urdf','*.xacro'))),

        (os.path.join('share',package_name,'world'),
         glob(os.path.join('world','*.world')))

    ] + model_files,

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kush',
    maintainer_email='kush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=[
        'scripts/follow_waypoints.py',
        'scripts/gotogoal.py',
    ],
)
