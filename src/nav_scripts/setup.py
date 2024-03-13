from setuptools import find_packages, setup
import rclpy

package_name = 'nav_scripts'

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
    maintainer='shuxy',
    maintainer_email='shuxy6263@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose = nav_scripts.nav_to_pose:main',
            'rotating = nav_scripts.rotating:main',
        ],
    },
)
