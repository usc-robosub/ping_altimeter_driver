from setuptools import setup

package_name = 'ping_altimeter_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arsalan Ghogari',
    maintainer_email='ghogari@usc.edu',
    description='ROS 2 driver for Blue Robotics Ping1D altimeter.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ping_altimeter_node = ping_altimeter_driver.ping_altimeter_node:main',
        ],
    },
)

