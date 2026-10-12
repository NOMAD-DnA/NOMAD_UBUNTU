from glob import glob
from setuptools import find_packages, setup
name = 'nomad_perception'
setup(name=name, version='0.2.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'README.md']),
                  ('share/'+name+'/config', glob('config/*')),
                  ('share/'+name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+name+'/docs', glob('docs/*.md')),
                  ('share/'+name+'/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='NOMAD planning team', maintainer_email='user@example.com',
      description='Replaceable NOMAD autonomy module', license='Apache-2.0',
      entry_points={'console_scripts': [
          'perception = nomad_perception.sensor_input:main',
          'semantic_points = nomad_perception.semantic_points_node:main',
          'geometry_mapper = nomad_geometry_mapping.mapper:main',
          'geometry_saved_map_viewer = nomad_geometry_mapping.viewer:main',
          'geometry_slope_labels = nomad_geometry_mapping.labels:main',
          'geometry_session = nomad_geometry_mapping.session:main']})
