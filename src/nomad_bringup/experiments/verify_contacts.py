#!/usr/bin/env python3
"""Small physics-only contact test. No user stack, camera or planner is changed."""
import os
from pathlib import Path
import subprocess
import tempfile
import time
import uuid
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from common import make_world,contact_classification,save
from experiment import stop_owned,check_domain


def main():
    os.environ['ROS_DOMAIN_ID']='189'
    os.environ['GZ_PARTITION']='nomad_contact_validation_'+uuid.uuid4().hex
    check_domain(189)
    directory=Path(tempfile.mkdtemp(prefix='nomad_contacts_'))
    sdf='''<sdf version="1.9"><world name="contact_validation">
<physics name="p" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<model name="terrain"><static>true</static><link name="ground"><collision name="ground"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision></link></model>
<model name="nomad_vehicle"><pose>0 0 0.3 0 0 0</pose><link name="base_link"><inertial><mass>1</mass><inertia><ixx>.01</ixx><iyy>.01</iyy><izz>.01</izz></inertia></inertial><collision name="body"><geometry><box><size>.4 .4 .4</size></box></geometry></collision></link></model></world></sdf>'''
    (directory/'original.sdf').write_text(sdf)
    topics=make_world(directory/'original.sdf',directory/'test.sdf')
    rclpy.init();node=rclpy.create_node('contact_pipeline_validation');observed=[]
    node.create_subscription(Contacts,topics[0],lambda m:observed.extend(contact_classification(c.collision1.name,c.collision2.name) for c in m.contacts),qos_profile_sensor_data)
    processes=[];streams=[]
    try:
        commands=[['ros2','run','ros_gz_bridge','parameter_bridge',topics[0]+'@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],['gz','sim','-s','-r',str(directory/'test.sdf')]]
        for i,cmd in enumerate(commands):
            stream=(directory/f'{i}.log').open('w');streams.append(stream)
            processes.append(subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+25
        while time.monotonic()<deadline and 'forbidden' not in observed:rclpy.spin_once(node,timeout_sec=.1)
        result=dict(passed='forbidden' in observed,classifications=sorted(set(observed)),messages=len(observed),logs=str(directory))
        save(directory/'result.json',result);print(result)
        if not result['passed']:raise RuntimeError('Actual Gazebo forbidden contact was not observed')
    finally:
        stop_owned(processes)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
