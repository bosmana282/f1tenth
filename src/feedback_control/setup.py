from setuptools import setup

package_name = 'feedback_control'

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
    description='Feedback controller for the f1tenth',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'feedback_control = feedback_control.feedback_control:main',
        ],
    },
)
