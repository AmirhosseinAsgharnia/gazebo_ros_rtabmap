from setuptools import find_packages, setup

package_name = 'arise_parrot_pkg'

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
    maintainer='amir',
    maintainer_email='amir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gazebo_launcher = arise_parrot_pkg.parrot_in_arise_gazebo:main",
            "odom_tf_publisher = arise_parrot_pkg.odom_tf_publisher_codes:main"
        ],
    },
)
