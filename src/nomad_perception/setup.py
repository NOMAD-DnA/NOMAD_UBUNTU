from glob import glob
from pathlib import Path
from setuptools import find_packages, setup
name = 'nomad_perception'
setup(name=name, version='0.2.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'README.md']),
                  ('share/'+name+'/config', [p for p in glob('config/*') if Path(p).is_file()]),
                  *[(str(Path('share')/name/p.parent),[str(p)]) for p in Path('config/real').rglob('*') if p.is_file()],
                  ('share/'+name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+name+'/docs', glob('docs/*.md')),
                  ('share/'+name+'/rviz', glob('rviz/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      package_data={'nomad_perception.real.ground_reference':['settings.yaml','provenance.json']},
      maintainer='NOMAD planning team', maintainer_email='user@example.com',
      description='Replaceable NOMAD autonomy module', license='Apache-2.0',
      entry_points={'console_scripts': [
          'real_perception = nomad_perception.real.node:main',
          'real_oak_capture = nomad_perception.real.oak_basalt_bridge:main',
          'perception = nomad_perception.sensor_input:main',
          'navigation_start = nomad_perception.navigation_start:main',
          'navigation_terrain = nomad_perception.navigation_terrain:main',
          'live_terrain = nomad_perception.live_terrain:main',
          'terrain_viewer = nomad_perception.terrain_viewer:main',
          'semantic_points = nomad_perception.semantic_points_node:main',
          'geometry_mapper = nomad_geometry_mapping.mapper:main',
          'geometry_saved_map_viewer = nomad_geometry_mapping.viewer:main',
          'geometry_slope_labels = nomad_geometry_mapping.labels:main',
          'geometry_session = nomad_geometry_mapping.session:main',
          'semantic_mapper = nomad_semantic_mapping.mapper:main',
          'semantic_saved_map_viewer = nomad_semantic_mapping.viewer:main',
          'semantic_labels = nomad_semantic_mapping.labels:main',
          'semantic_session = nomad_semantic_mapping.session:main']})
