from setuptools import find_packages, setup

package_name = 'frame_servo_control'

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
    description='Dual RDS51150 servo control from joystick input.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_servo_joy = frame_servo_control.dual_servo_joy_node:main',
        ],
    },
)
