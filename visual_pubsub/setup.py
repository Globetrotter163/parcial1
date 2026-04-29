from setuptools import setup
import os
from glob import glob

package_name = 'visual_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alan',
    maintainer_email='alan.gonzales@ucb.edu.bo',
    description='cinemática inversa ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_joints         = visual_pubsub.pub_joints:main',
            'sub_joints         = visual_pubsub.sub_joints:main',
            'inverse_kinematics = visual_pubsub.inverse_kinematics:main',
        ],
    },
)