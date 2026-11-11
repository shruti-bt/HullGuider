from setuptools import find_packages, setup

package_name = 'hull_guider'

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
    maintainer='marvl',
    maintainer_email='marvl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dvl_hull = hull_guider.core.dvl_hull_node:main',
            'guided_controller = hull_guider.core.guided_controller:main'
            
        ],
    },
)
