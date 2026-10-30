from setuptools import setup

package_name = 'nomad_operator_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/boards.env.example']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NOMAD',
    maintainer_email='user@example.com',
    description='PC operator view for NOMAD camera, costmap, and goal selection',
    license='Apache-2.0',
    entry_points={'console_scripts': ['operator_gui = nomad_operator_gui.app:main']},
)
