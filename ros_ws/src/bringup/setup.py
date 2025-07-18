# bringup/setup.py
from glob import glob
from setuptools import find_packages, setup
from pathlib import Path

package_name = 'bringup'
share_dir = f'share/{package_name}'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (share_dir, ['package.xml']),

        # --- runtime assets ------------------------------------------------
        (f'{share_dir}/launch',  glob('launch/*.launch.py')),
        (f'{share_dir}/config',  glob('config/*')),
        (f'{share_dir}/rviz',    glob('rviz/*.rviz')),
        (f'{share_dir}/worlds',  glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selfsim',
    maintainer_email='jonas.buro1@gmail.com',
    description='Entry point to sim/real',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # eg. 'teleop=bringup.scripts.teleop:main',
        ],
    },
)
