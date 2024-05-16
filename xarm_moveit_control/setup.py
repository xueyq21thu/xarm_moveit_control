from setuptools import find_packages, setup

package_name = 'xarm_moveit_control'

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
    maintainer='lgl',
    maintainer_email='1165843210@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "xarm_moveit_control_service = xarm_moveit_control.xarm_moveit_control_service:main",
            "add_table = xarm_moveit_control.add_table:main",
        ],
    },
)
