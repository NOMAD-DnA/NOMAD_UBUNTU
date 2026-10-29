"""Geometry, physical model compatibility, and command-loss regression tests."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
SENSORS=Path(__file__).resolve().parents[1]/'config/sensors.yaml'
sys.path.insert(0,str(SCRIPTS))
from rear_steering_model import Geometry,CommandGate
import generate_vehicle as generator


class KinematicsTests(unittest.TestCase):
    def test_no_side_slip_at_all_four_wheels(self):
        g=Geometry()
        for speed in (-.4,.4):
            for alpha in (-.4,-.1,0.,.1,.4):
                wl,wr,dl,dr=g.mix(speed,alpha)
                yaw=speed*math.tan(alpha)/g.wheelbase
                for x,y,delta in ((g.wheelbase/2,g.track/2,0.),(g.wheelbase/2,-g.track/2,0.),
                                   (-g.wheelbase/2,g.track/2,dl),(-g.wheelbase/2,-g.track/2,dr)):
                    vx=speed-y*yaw;vy=(x-g.wheelbase/2)*yaw
                    self.assertAlmostEqual(-vx*math.sin(delta)+vy*math.cos(delta),0.,places=12)
                self.assertAlmostEqual(wl*g.radius,speed-yaw*g.track/2)
                self.assertAlmostEqual(wr*g.radius,speed+yaw*g.track/2)
                self.assertAlmostEqual(g.equivalent_from_joints(dl,dr),alpha)

    def test_left_turn_and_reverse(self):
        g=Geometry();forward=g.mix(.4,.3);reverse=g.mix(-.4,.3)
        self.assertLess(forward[0],forward[1])
        self.assertLess(forward[2],0.);self.assertLess(forward[3],0.)
        self.assertGreater(abs(forward[2]),abs(forward[3]))
        self.assertEqual(reverse[2:],forward[2:])
        self.assertAlmostEqual(reverse[0],-forward[0])
        # Centre initially swings outside the turn, rather than following rear-axle FWS geometry.
        end=g.advance((0.,0.,0.),.4,.3,.01)
        self.assertGreater(end[2],0.);self.assertLess(end[1],0.)
        back=g.advance(end,-.4,.3,.01)
        for value in back:self.assertAlmostEqual(value,0.,places=12)

    def test_inner_joint_limits_and_stationary_steering(self):
        g=Geometry()
        for alpha in (-g.equivalent_limit,g.equivalent_limit):
            values=g.mix(0.,alpha)
            self.assertEqual(values[:2],(0.,0.))
            self.assertAlmostEqual(max(map(abs,values[2:])),g.joint_limit)
        with self.assertRaises(ValueError):g.mix(.2,g.equivalent_limit+.001)
        with self.assertRaises(ValueError):g.mix(float('nan'),0.)


class GateTests(unittest.TestCase):
    def send(self,gate,**kwargs):
        values=dict(speed=.3,angle=.2,stamp=10.,lease=.3,stop=False,frame='base_link',now=10.,wall=20.)
        values.update(kwargs);gate.receive(**values)

    def test_wall_timeout_even_when_simulation_paused(self):
        g=CommandGate();self.send(g)
        self.assertEqual(g.sample(10.,20.1),(.3,.2))
        self.assertEqual(g.sample(10.,20.36),(0.,0.))

    def test_duplicate_does_not_renew_and_same_tick_stop_wins(self):
        g=CommandGate();self.send(g);self.send(g,wall=20.3)
        self.assertEqual(g.sample(10.,20.36),(0.,0.))
        self.send(g,stamp=10.1,now=10.1,wall=20.4)
        self.send(g,stamp=10.1,now=10.1,wall=20.4,stop=True)
        self.assertEqual(g.sample(10.1,20.4),(0.,0.))
        self.send(g,stamp=10.1,now=10.1,wall=20.45)
        self.assertEqual(g.sample(10.1,20.45),(0.,0.))

    def test_invalid_inputs_and_ros_expiry(self):
        for override in ({'speed':2.},{'angle':1.},{'angle':float('nan')},
                         {'frame':'odom'},{'lease':1.},{'stamp':9.},{'stamp':11.}):
            g=CommandGate();self.send(g,**override)
            self.assertEqual(g.sample(10.,20.),(0.,0.))
        g=CommandGate();self.send(g)
        self.assertEqual(g.sample(10.31,20.1),(0.,0.))

    def test_clock_reset_latches_and_old_stop_cannot_override_new_command(self):
        g=CommandGate();self.send(g);g.sample(10.,20.)
        self.send(g,stamp=10.1,now=10.1,wall=20.1)
        self.send(g,stop=True,stamp=10.,now=10.1,wall=20.1)
        self.assertEqual(g.sample(10.1,20.1),(.3,.2))
        self.assertEqual(g.sample(1.,20.2),(0.,0.))
        self.send(g,stamp=11.,now=11.,wall=20.3)
        self.assertEqual(g.sample(11.,20.3),(0.,0.))


class ModelTests(unittest.TestCase):
    def test_rear_physical_structure_and_urdf_agree(self):
        links,joints=generator.physical_model('front_drive_rear_steer')
        p=generator.sensor_parameters(generator.load_sensors(SENSORS))
        frames=generator.fixed_frames(p)
        sdf=generator.make_sdf(links,joints,frames,p,False,'front_drive_rear_steer').find('model')
        urdf=generator.make_urdf(links,joints,frames)
        self.assertEqual(len(sdf.findall("plugin[@name='gz::sim::systems::JointController']")),2)
        self.assertEqual(len(sdf.findall("plugin[@name='gz::sim::systems::JointPositionController']")),2)
        self.assertIsNone(sdf.find("plugin[@name='gz::sim::systems::AckermannSteering']"))
        for side in ('left','right'):
            self.assertIsNone(sdf.find(f"joint[@name='front_{side}_steering_joint']"))
            for name,parent,axis in ((f'front_{side}_wheel_joint','base_link','0 1 0'),
                (f'rear_{side}_steering_joint','base_link','0 0 1'),
                (f'rear_{side}_wheel_joint',f'rear_{side}_steering_link','0 1 0')):
                sj=sdf.find(f"joint[@name='{name}']");uj=urdf.find(f"joint[@name='{name}']")
                self.assertEqual(sj.findtext('parent'),parent)
                self.assertEqual(uj.find('parent').get('link'),parent)
                self.assertEqual(sj.findtext('axis/xyz'),axis)
                self.assertEqual(uj.find('axis').get('xyz'),axis)

    def test_default_vehicle_remains_front_steering(self):
        links,joints=generator.physical_model()
        names={j['name'] for j in joints}
        self.assertIn('front_left_steering_joint',names)
        self.assertNotIn('rear_left_steering_joint',names)
        with self.assertRaises(ValueError):generator.physical_model('typo')

    def test_rear_metadata_does_not_advertise_removed_ackermann_topics(self):
        with tempfile.TemporaryDirectory() as directory:
            m=generator.generate_vehicle(Path(directory),SENSORS,with_sensors=False,drive_layout='front_drive_rear_steer')
        self.assertEqual(m['driven_joints'],['front_left_wheel_joint','front_right_wheel_joint'])
        self.assertNotIn('wheel_odom',m['native_topics'])
        self.assertNotIn('cmd_vel',m['native_topics'])
        self.assertEqual(len(m['actuator_topics']),4)
        self.assertIn('wheel_odom',generator.TOPICS)  # No mutation of the legacy profile.


if __name__=='__main__':unittest.main()
