from setuptools import find_packages, setup

package_name = 'nikon_mtplv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nikon_mtplv.launch.py']),
        ('share/' + package_name + '/scripts', [
            'scripts/nikon_usb_recover.sh',
            'scripts/start_mtplvcap_d3500.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frame',
    maintainer_email='frame@todo.todo',
    description='Nikon MTP LiveView ROS2 bridge.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mtplvcap_stream = nikon_mtplv.mtplvcap_stream_node:main',
        ],
    },
)
