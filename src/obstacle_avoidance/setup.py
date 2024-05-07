from setuptools import setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rma-orin',
    maintainer_email='andries.bosman@telenet.be',
    description='Obstacle avoidance algorithm for the f1tenth',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'waypoint_navigation = obstacle_avoidance.waypoint_navigation:main',
        ],
    },
)
