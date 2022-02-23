from setuptools import setup

package_name = 'flex_bt_flexbe_states'

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
    maintainer='Joshua Zutell',
    maintainer_email='joshua.zutell.18@cnu.edu',
    description='flex_bt_flexbe_states provides a collection of FlexBE states used during the SSP Research',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
