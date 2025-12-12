from setuptools import setup

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/state_estimation']),
    ('share/state_estimation', ['package.xml']),
    ('share/state_estimation/config', ['config/ekf.yaml']),
    ('share/state_estimation/launch', ['launch/ekf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rlspeed',
    maintainer_email='subratpr001@e.ntu.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
