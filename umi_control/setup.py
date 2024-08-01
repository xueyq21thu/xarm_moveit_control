from setuptools import find_packages, setup

package_name = 'umi_control'

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
    maintainer='robot1',
    maintainer_email='1440318621@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper = umi_control.gripper:main'
            'tcp_socket_plan = umi_control.tcp_socket_plan:main',
            'tcp_socket_move = umi_control.tcp_socket_move:main',
        ],
    },
)
