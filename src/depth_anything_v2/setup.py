from setuptools import setup, find_packages

package_name = 'depth_anything_v2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),   # <-- THIS is the fix
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # <-- ok to keep
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='DepthAnythingV2 ROS2 integration for real-time depth estimation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_anything_v2_node = depth_anything_v2.depth_anything_node:main',
        ],
    },
)
