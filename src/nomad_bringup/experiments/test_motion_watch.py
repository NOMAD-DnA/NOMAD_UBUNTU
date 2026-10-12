from motion_watch import MotionWatch


def test_stationary_noise_ends_at_sim_threshold():
    watch=MotionWatch(10.,[0.,0.])
    assert not watch.update(69.99,[.01,-.01])
    assert watch.update(70.,[.01,-.01])


def test_reverse_motion_resets_timeout():
    watch=MotionWatch(0.,[0.,0.])
    assert not watch.update(59.,[-.51,0.])
    assert not watch.update(118.,[-.51,0.])
    assert watch.update(119.,[-.51,0.])


def test_paused_clock_does_not_expire():
    watch=MotionWatch(0.,[0.,0.])
    for _ in range(1000):
        assert not watch.update(1.,[0.,0.])
