from setuptools import setup
import os
from glob import glob

package_name = 'cartographer_config'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # cartographer config (lua files)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.lua')),
        
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),

        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edupez05',
    maintainer_email='edupez05@example.com',
    description='Cartographer configuration for custom differential drive robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
