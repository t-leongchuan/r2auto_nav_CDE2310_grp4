from setuptools import find_packages, setup

package_name = 'robot_fsm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turdle',
    maintainer_email='t-leongchuan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor = robot_fsm_controller.supervisor:main',
            'exploration = robot_fsm_controller.exploration:main',
            'alignment = robot_fsm_controller.alignment:main',
            'pursuit = robot_fsm_controller.pure_pursuit:main',
            'frontier = robot_fsm_controller.frontier_exploration:main',
            'occgrid = robot_fsm_controller.occgrid_test:main',
            'random = robot_fsm_controller.frontier_exploration_test:main',
            'thermal_detect = robot_fsm_controller.thermal_target:main',
            'firing = robot_fsm_controller.firing:main',
        ],
    },
)
