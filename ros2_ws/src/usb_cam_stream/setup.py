from setuptools import find_packages, setup

package_name = 'usb_cam_stream'

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
    description='USB UVC camera streaming node for capture cards.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_cam_stream = usb_cam_stream.usb_cam_stream_node:main',
        ],
    },
)
