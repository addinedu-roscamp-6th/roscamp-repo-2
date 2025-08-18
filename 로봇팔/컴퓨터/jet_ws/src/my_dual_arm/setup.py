from setuptools import setup, find_packages

package_name = 'my_dual_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetcobot',
    maintainer_email='you@example.com',
    description='Robot A and B coordination system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_a1_node = my_dual_arm.robot_a1_node:main',
            'robot_b1_node = my_dual_arm.robot_b1_node:main',
            'coordinator_node = my_dual_arm.coordinator_node:main',
        ],
    },
)
