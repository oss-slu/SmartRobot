from setuptools import find_packages, setup

package_name = 'slu_smart'

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
    maintainer='Pascal Sikorski',
    maintainer_email='pascal.sikorski@slu.edu',
    description='SLU_SMART ROS2 package for all software of the SLU SMART Robot. Currently an example hello world publisher is active',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = slu_smart.test_pub:main',
        ],
    },
)
