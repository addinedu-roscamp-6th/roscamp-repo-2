from setuptools import setup, find_packages

package_name = 'my_dual_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Dual arm control package',
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
