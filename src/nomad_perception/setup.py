from glob import glob
from setuptools import find_packages, setup
name = 'nomad_perception'
setup(name=name, version='0.2.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'README.md']),
                  ('share/'+name+'/config', glob('config/*')),
                  ('share/'+name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+name+'/docs', glob('docs/*.md'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='NOMAD planning team', maintainer_email='user@example.com',
      description='Replaceable NOMAD autonomy module', license='Apache-2.0',
      entry_points={'console_scripts': ['perception = nomad_perception.sensor_input:main']})
