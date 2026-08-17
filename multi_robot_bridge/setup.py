import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Mr',
    maintainer_email='you@example.com',
    description=(
        'Cross-domain relay + TF publisher bridging the Tello (cyclonedds, '
        'domain 10) and ASTRO (rmw_zenoh_cpp) ROS 2 graphs.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_relay = multi_robot_bridge.process_a_tello_side:main',
            'drone_tf_publisher = multi_robot_bridge.process_b_astro_side:main',
        ],
    },
)
