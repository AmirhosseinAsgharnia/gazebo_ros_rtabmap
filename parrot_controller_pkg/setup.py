from setuptools import find_packages, setup

package_name = 'parrot_controller_pkg'

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
            "full_drone_controller = parrot_controller_pkg.parrot_control_node:main",
            "take_off_controller = parrot_controller_pkg.take_off_node:main",
            "bebop_cmd=parrot_controller_pkg.parrot_cmd_codes:main"
        ],
    },
)
