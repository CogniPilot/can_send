from setuptools import setup

package_name = 'can_send'

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
    author='Cogni',
    author_email='info@cognipilot.com',
    maintainer='Cogni',
    maintainer_email='info@cognipilot.com',
    description='ROS 2 node for sending CAN frames over socketcan.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "can_send_node = can_send.can_send_node:main"
        ],
    },
)
