from setuptools import find_packages, setup

package_name = 'frame_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/frame_all.launch.py',
            'launch/frame_all_stream_only.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/teleop_twist_joy_xbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frame',
    maintainer_email='frame@todo.todo',
    description='Bringup launch files and utilities for Frame robot deployments.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_record_toggle = frame_bringup.joy_record_toggle_node:main',
        ],
    },
)
