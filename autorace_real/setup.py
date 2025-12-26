from setuptools import find_packages, setup

package_name = 'autorace_real'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enter_tunnel_client_request = autorace_real.enter_tunnel_client_request:main',
            'tunnel_navigation_srv = autorace_real.tunnel_navigation_srv:main',
            'obstacle_avoid_navigation = autorace_real.obstacle_avoid_navigation:main'
            'follow_both_line = autorace_real.follow_both_line:main'
        ],
    },
)
