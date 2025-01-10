from setuptools import find_packages, setup

package_name = 'subjugator_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vision_stack'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel27parra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vs_test = subjugator_perception.nodes.vs_test:main'
        ],
    },
)
