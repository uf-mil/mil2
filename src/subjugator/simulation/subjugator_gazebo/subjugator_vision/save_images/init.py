from setuptools import setup

package_name = 'subjugator_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    data_files=[
        # Makes the package discoverable by ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Installs your package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',          # standard
        'opencv-python'        # gives you `cv2`
    ],
    zip_safe=True,
    maintainer='Carlos Chavez',
    maintainer_email='carlos@example.com',
    description='Image-saving node for Subjugator simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run subjugator_vision image_saver
            'image_saver = ' + package_name + '.image_saver:main',
        ],
    },
)
