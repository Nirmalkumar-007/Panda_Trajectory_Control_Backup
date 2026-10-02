import os
import subprocess
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = 'panda_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iit-rain',
    maintainer_email='iit-rain@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PandaCmdSub = app.run_PandaCmdSub:main', 
            'PandaTrajectoryNode = app.run_PandaTrajectory:main',
            'PandaTrapNode = app.run_PandaTrap:main',
            'test_PandaCmdSub = test.test_pandaCmdSub:test_pandacmdsub',
            'test_PandaTrajectoryNode = test.test_PandaTrajectory:test_pandatrajectory',

        ],
    },
)



