import unittest
import numpy as np
from nomad_perception.real.vio_health import PoseHealth,stationary_metrics


class Health(unittest.TestCase):
    def test_bad_trajectory_is_latched_and_normal_walking_allowed(self):
        health=PoseHealth()
        for i in range(100):health.observe([i*33_333_333,i*.03,0,0,0,0,0,1])
        self.assertIsNone(health.error)
        for i in range(100,106):health.observe([i*33_333_333,1000+i,0,0,0,0,0,1])
        self.assertIsNotNone(health.error)
        health.observe([107*33_333_333,1105,0,0,0,0,0,1])
        self.assertIsNotNone(health.error)

    def test_two_seconds_and_unchanged_stationarity_threshold(self):
        rows=[[i*33_333_333,.005*np.sin(i),0,0,0,0,0,1]for i in range(61)]
        a=stationary_metrics(rows);self.assertTrue(a['ready']);self.assertLess(a['translation_m'],.03)
        self.assertFalse(stationary_metrics(rows[:5])['ready'])
        rows[-1][1]=.1
        self.assertGreater(stationary_metrics(rows)['translation_m'],.03)

if __name__=='__main__':unittest.main(verbosity=2)
