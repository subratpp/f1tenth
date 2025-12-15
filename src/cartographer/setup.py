import os
from glob import glob
from setuptools import setup

package_name = 'cartographer' # TODO: Update this to match your package.xml name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # INSTALL LAUNCH FILES
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # INSTALL CONFIG FILES (LUA)
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f1tenth',
    maintainer_email='f1tenth@todo.todo',
    description='F1TENTH Cartographer SLAM Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add python node entry points here if you have any
        ],
    },
)