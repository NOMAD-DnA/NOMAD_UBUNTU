from glob import glob
from setuptools import find_packages, setup

package_name = 'nomad_path_planning'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'VALIDATION.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='NOMAD planning team', maintainer_email='user@example.com',
    description='NOMAD sensor-fed D* Lite, Ackermann local planning and reverse recovery',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'recovery_supervisor = nomad_path_planning.recovery_supervisor:main',
        'dstar_lite_gpp = nomad_path_planning.recovery_gpp:main',
        'ackermann_lpp = nomad_path_planning.local_planner_node:main',
        'planning_inputs = nomad_path_planning.input_bridge:main',
        'planning_command = nomad_path_planning.command_selector:main',
    ]},
)
