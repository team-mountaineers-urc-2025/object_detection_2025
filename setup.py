from setuptools import find_packages, setup

package_name = 'object_detect'

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
    maintainer='Izaak',
    maintainer_email='izaakwhetsell@gmail.com',
    description='Implements YOLO model for detection of hammer and water bottle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_distance = object_detect.object_distance:main'
            
        ],
    },
)
