from setuptools import setup

package_name = 'farm_robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rutha',
    maintainer_email='rutha@example.com',
    description='Crop stress detection node using OpenCV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_stress_node = farm_robot_perception.crop_stress_node:main',
        ],
    },
)
