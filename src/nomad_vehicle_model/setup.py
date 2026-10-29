from setuptools import setup
setup(name='nomad_vehicle_model', version='0.1.0', packages=['nomad_vehicle_model'],
 data_files=[('share/ament_index/resource_index/packages',['resource/nomad_vehicle_model']),
 ('share/nomad_vehicle_model',['package.xml','README.md'])],
 install_requires=['setuptools'], license='Apache-2.0', maintainer='NOMAD',
 maintainer_email='user@example.com', description='Shared planar vehicle geometry')
