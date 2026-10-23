from setuptools import setup

package_name = 'my_rosbot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manel Puig',
    maintainer_email='manel.puig@ub.edu',
    description='Parameterized planar velocity tests for the ROSbot XL',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drive_test = my_rosbot_control.drive_test:main',
            'interface_adapter = my_rosbot_control.interface_adapter:main',
        ],
    },
)
