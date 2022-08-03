import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'flex_bt_flexbe_states'
submodules = "flex_bt_flexbe_states/utility"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    data_files=[
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,"test"), glob('test/*.test')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Zutell',
    maintainer_email='joshua.zutell.18@cnu.edu',
    description='flex_bt_flexbe_states provides a collection of FlexBE states used with flexible behavior trees',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
