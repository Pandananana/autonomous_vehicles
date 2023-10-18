import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aw',
    maintainer_email='andreaswendelboe.dk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow = wall_follow.sub_scan_pub_pcloud:main',
        ],
    },
)
