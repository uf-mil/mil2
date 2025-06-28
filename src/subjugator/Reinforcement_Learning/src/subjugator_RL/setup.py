import glob
from setuptools import find_packages, setup

package_name = 'subjugator_RL'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['subjugator_RL', 'subjugator_RL.*']),
    include_package_data=True,             # pick up any package data if needed
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='RL environment nodes for Subjugator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sub_RL_train = subjugator_RL.sub_RL_train:main',
        ],
    },
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml in share/<pkg>
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.py')),
        # urdf files
        ('share/' + package_name + '/urdf',
            glob.glob('urdf/*.xacro')),
        # world files
        ('share/' + package_name + '/worlds',
            glob.glob('worlds/*.world')),
    ],
)
