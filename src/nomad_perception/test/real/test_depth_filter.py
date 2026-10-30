import unittest
import numpy as np
from nomad_perception.real.depth_filter import FilteredDepth, FilterSettings, OnlineDepthFilter, frame_voxels


class DepthFilterTest(unittest.TestCase):
    def test_many_pixels_in_one_frame_cannot_confirm_themselves(self):
        f = FilteredDepth()
        p = np.repeat([[.021, .021, 2.021]], 100, axis=0)
        f.add_frame(p, p, 1)
        self.assertEqual(len(f.arrays()[0]), 0)
        with self.assertRaises(ValueError):
            f.add_frame(p, p, 1)

    def test_noisy_plane_transient_cluster_and_far_returns(self):
        rng = np.random.default_rng(72)
        x, y = np.meshgrid(np.arange(12)*.05+.025, np.arange(12)*.05+.025)
        plane = np.c_[x.ravel(), y.ravel(), np.full(x.size, 2.025)]
        transient = plane + [1.5, 0, 0]
        far = plane + [0, 0, 3]
        f = FilteredDepth()
        errors = []
        for frame in range(10):
            noisy = plane + rng.normal(0, .006, plane.shape)
            errors.extend(noisy[:, 2] - 2.025)
            points = np.r_[noisy, far, transient if frame == 0 else np.empty((0, 3)),
                           [[-10, -10, 2], [8, 8, 3]]]
            f.add_frame(points, points, frame)
        p, counts = f.arrays()
        self.assertGreater(len(p), 100)
        self.assertTrue(np.all(counts >= 2))
        self.assertTrue(np.all(p[:, 0] < 1))  # Transient and isolated returns gone.
        self.assertTrue(np.all(p[:, 2] < 3))  # Repeated out-of-range returns gone.
        self.assertLess(np.sqrt(np.mean((p[:, 2]-2.025)**2)), np.std(errors)*.65)

    def test_equal_frame_weight_and_separate_surfaces(self):
        settings = FilterSettings(min_neighbors=1)
        f = FilteredDepth(settings)
        base = np.array([[.01, .01, 2.01], [.061, .01, 2.01],
                         [.01, .01, 2.21], [.061, .01, 2.21]])
        f.add_frame(np.repeat(base, 100, axis=0), np.repeat(base, 100, axis=0), 1)
        other = base + [.02, 0, .02]
        f.add_frame(other, other, 2)
        p, n = f.arrays()
        self.assertEqual(len(p), 4)
        expected = base + [.01, 0, .01]
        expected = expected[np.lexsort((expected[:, 2], expected[:, 0]))]
        np.testing.assert_allclose(p, expected)
        np.testing.assert_array_equal(n, [2]*4)


class OnlineDepthFilterTest(unittest.TestCase):
    def patch(self, offset=0., z=2.025):
        x, y = np.meshgrid(np.arange(8)*.05+.025+offset, np.arange(8)*.05+.025)
        return np.c_[x.ravel(), y.ravel(), np.full(x.size, z)]

    def test_causal_confirmation_and_original_frame_positions(self):
        online = OnlineDepthFilter()
        first = self.patch()
        self.assertEqual(len(online.add_frame(first, first, 1_000_000_000)['points_world_m']), 0)
        second = first + [.004, 0., .004]
        result = online.add_frame(second, second, 1_500_000_000)
        self.assertGreater(len(result['points_world_m']), 20)
        # Keep actual current-frame centroids, rather than stale temporal means.
        self.assertTrue(np.allclose(result['points_world_m'][:, 2], 2.029))
        np.testing.assert_array_equal(result['support_frames'], 2)
        np.testing.assert_array_equal(result['support_first_stamp_ns'], 1_000_000_000)
        np.testing.assert_array_equal(result['support_last_stamp_ns'], 1_500_000_000)
        with self.assertRaises(ValueError):
            online.add_frame(second, second, 1_500_000_000)

    def test_legacy_support_equivalence_without_cumulative_replay(self):
        old, online = FilteredDepth(), OnlineDepthFilter()
        a = self.patch()
        for t in (1_000_000_000, 1_500_000_000):
            old.add_frame(a, a, t)
            result = online.add_frame(a, a, t)
        np.testing.assert_allclose(result['points_world_m'], old.arrays()[0])
        # Old region remains in the filter's history, but a new camera view
        # must not send it again or give it the new capture timestamp.
        b = self.patch(offset=2.)
        self.assertEqual(len(online.add_frame(b, b, 2_000_000_000)['points_world_m']), 0)
        result = online.add_frame(b, b, 2_500_000_000)
        self.assertTrue(np.all(result['points_world_m'][:, 0] > 2.))
        self.assertGreater(online.report()['confirmed_voxels'], len(result['points_world_m']))

    def test_expired_observations_cannot_confirm_a_new_capture(self):
        online = OnlineDepthFilter()
        a = self.patch()
        online.add_frame(a, a, 1_000_000_000)
        self.assertEqual(len(online.add_frame(a, a, 12_000_000_000)['points_world_m']), 0)
        result = online.add_frame(a, a, 12_500_000_000)
        self.assertGreater(len(result['points_world_m']), 20)
        np.testing.assert_array_equal(result['support_first_stamp_ns'], 12_000_000_000)

    def test_range_and_sor_and_empty_frames(self):
        online = OnlineDepthFilter()
        a = self.patch()
        far = a + [0, 0, 4]
        points = np.r_[a, far, [[20, 20, 2.025]]]
        for stamp in (1_000_000_000, 1_500_000_000):
            result = online.add_frame(points, points, stamp)
        self.assertTrue(np.all(result['points_world_m'][:, 0] < 1.))
        self.assertTrue(np.all(result['points_world_m'][:, 2] < 4.))
        self.assertEqual(len(online.add_frame(np.empty((0, 3)), np.empty((0, 3)), 2_000_000_000)['points_world_m']), 0)
        self.assertGreater(online.report()['input_points'], online.report()['range_kept_points'])

    def test_same_frame_pixels_never_count_as_temporal_support(self):
        online = OnlineDepthFilter()
        a = np.repeat(self.patch(), 100, axis=0)
        result = online.add_frame(a, a, 1_000_000_000)
        self.assertEqual(len(result['points_world_m']), 0)

    def test_depth_range_uses_camera_z_when_world_ground_is_zero(self):
        online = OnlineDepthFilter()
        world = self.patch(z=0.)
        camera = world + [0., 0., 1.2]
        online.add_frame(camera, world, 1_000_000_000)
        result = online.add_frame(camera, world, 1_500_000_000)
        self.assertGreater(len(result['points_world_m']), 20)
        self.assertTrue(np.all(result['points_world_m'][:, 2] == 0.))


if __name__ == '__main__':
    unittest.main()
