from setuptools import find_packages, setup

package_name = 'gst_communicator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frame',
    maintainer_email='frame@todo.todo',
    description='ROS 2 bridge for gst_recording_daemon control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gst_communicator = gst_communicator.gst_communicator_node:main',
        ],
    },
)
