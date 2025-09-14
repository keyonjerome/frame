from setuptools import find_packages, setup

package_name = 'dancer_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follow_person.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frame',
    maintainer_email='frame@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dancer_seg = dancer_detector.dancer_seg_node:main',
            'yolo_ir_realsense = dancer_detector.yolo_ir_realsense_node:main',
            'person_follower = dancer_detector.person_follower_pid:main',
            'uart_seg_toggle = dancer_detector.uart_seg_toggle:main',
                    'centroid_to_centerdepth = dancer_detector.centroid_to_centerdepth:main',

                                ],
    },
)
