from setuptools import find_packages, setup

package_name = 'gps_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'gps_navigation.openrouteservice_navigation_api',
        'gps_navigation.zed_f9p',
        'gps_navigation.purepursuit'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon',
    maintainer_email='moon@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_listener_node = gps_navigation.openrouteservice_navigation_api:main',
            'current_gps_node = gps_navigation.zed_f9p:main',
            'pure_pursuit_node = gps_navigation.purepursuit:main',
        ],
    },
)
