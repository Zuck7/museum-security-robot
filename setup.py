import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'museum_security_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Every non-Python asset has to be listed here or it will not end up in
        # install/share, and get_package_share_directory() will not find it at
        # runtime.  This is the single most common "it worked before I built
        # it" mistake in ament_python packages.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'prompts'), glob('prompts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='you@example.com',
    description='Museum security robot: Gazebo + Nav2 + LLM command interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'museum_ai_navigator = '
            'museum_security_robot.museum_ai_navigator:main',
            'odom_tf_broadcaster = '
            'museum_security_robot.odom_tf_broadcaster:main',
        ],
    },
)
