from setuptools import find_packages, setup

package_name = 'karl_navigation'

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
    maintainer='tobiasnat',
    maintainer_email='tobias.weyer@tngtech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive2 = karl_navigation.drive2:main',
            'nav2room = karl_navigation.nav_to_room:main'
        ],
    },
)
