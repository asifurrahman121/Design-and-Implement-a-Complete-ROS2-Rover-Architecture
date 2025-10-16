from setuptools import setup
from glob import glob

package_name = 'urc_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asifur-rahman',
    maintainer_email='asifurr477@gmail.com',
    description='URC rover nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mission_manager = urc_rover.mission_manager:main',
            'delivery_action_server = urc_rover.delivery_action_server:main',
            'delivery_action_client = urc_rover.delivery_action_client:main',
            'science_node = urc_rover.science_node:main',
        ],
    },
)

