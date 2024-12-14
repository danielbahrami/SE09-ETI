from setuptools import find_packages, setup

package_name = 'mir_data_collection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokas',
    maintainer_email='black.mad.eagle@gmail.com',
    description='Experts in Teams project for data collection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_monitor = mir_data_collection.sensor_monitor:main',
        ],
    },
)
