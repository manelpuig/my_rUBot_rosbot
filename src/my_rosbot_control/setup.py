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
    maintainer_email='manel@ub.edu',
    description='Minimal ROSbot XL control package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_simple = my_rosbot_control.teleop_simple:main',
        ],
    },
)
