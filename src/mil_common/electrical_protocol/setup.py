from setuptools import find_packages, setup

package_name = 'electrical_protocol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'serial'],
    zip_safe=True,
    maintainer='Cameron Brown',
    maintainer_email='me@cbrxyz.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculator = electrical_protocol.examples.calculator_device:main',
        ],
    },
)
