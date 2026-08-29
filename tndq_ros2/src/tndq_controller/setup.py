from setuptools import find_packages, setup

package_name = 'tndq_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tndq_control.launch.py']),
        ('share/' + package_name + '/config', ['config/tndq_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljn',
    maintainer_email='ljn@localhost',
    description=(
        'TNDQ torque-level feedback control over the official rebotarm '
        'ROS2 passthrough channel'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tndq_controller = tndq_controller.control_node:main',
        ],
    },
)
