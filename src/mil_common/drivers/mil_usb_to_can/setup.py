from setuptools import find_packages, setup

package_name = 'mil_usb_to_can'

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
    maintainer='Daniel Parra',
    maintainer_email='daniel27parra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sub9_driver = mil_usb_to_can.sub9.sub9_driver:main"
        ],
    },
)
