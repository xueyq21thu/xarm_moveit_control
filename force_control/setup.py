from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'force_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot1',
    maintainer_email='596081082@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ft_vis = force_control.ft_vis:main",
            "fc = force_control.force_control:main",
            "ft_pub = force_control.ft_pub:main",
            "fsm = force_control.finite_state_machine:main",
        ],
    },
)
