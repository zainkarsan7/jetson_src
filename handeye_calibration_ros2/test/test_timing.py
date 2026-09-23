from threading import Event

import pytest

from handeye_calibration_ros2.calibration import Sample, check_capture
from handeye_calibration_ros2.geometry import transform
from handeye_calibration_ros2.timing import LatestWorker, check_age


def test_delayed_low_rate_stationary_observations_are_usable():
    # 1.67 observations/s, with the latest image received two seconds late.
    history = [Sample(transform(), transform(translation=[0, 0, 1]), int(t*1e9))
               for t in (1.0, 1.6, 2.2)]
    assert check_capture(history, [], 4200000000, max_age_s=5, settle_s=1,
                         max_gap_s=1) is history[-1]
    # Sparse sampling must not bypass motion checks.
    history[1].base_from_effector = transform(translation=[0.02, 0, 0])
    with pytest.raises(ValueError, match='moving'):
        check_capture(history, [], 4200000000, max_age_s=5, settle_s=1, max_gap_s=1)


def test_long_observation_gap_still_rejected():
    history = [Sample(transform(), transform(), int(t*1e9)) for t in (1, 1.1, 3)]
    with pytest.raises(ValueError, match='largest gap 1.900s'):
        check_capture(history, [], 3100000000, max_age_s=5, settle_s=1, max_gap_s=1)


def test_age_errors_distinguish_delay_from_future_clock():
    assert check_age(1000000000, 3000000000, 5) == 2
    with pytest.raises(ValueError, match='age 6.000s exceeds 5.000s'):
        check_age(1000000000, 7000000000, 5)
    with pytest.raises(ValueError, match='0.200s ahead'):
        check_age(1200000000, 1000000000, 5)


def test_latest_worker_discards_backlog_without_parallel_detection():
    started, release = Event(), Event()
    calls = []

    def slow(value):
        calls.append(value)
        if value == 1:
            started.set()
            assert release.wait(timeout=3)
        return value*2

    worker = LatestWorker(slow)
    try:
        worker.put(1)
        assert worker.start()
        assert started.wait(timeout=3)
        for i in (2, 3, 4):
            worker.put(i)
        assert not worker.start()
        assert worker.replaced == 2
        release.set()
        worker.running[1].result(timeout=3)
        assert worker.take_done() == (1, 2, None)
        assert worker.start()
        worker.running[1].result(timeout=3)
        assert worker.take_done() == (4, 8, None)
        assert calls == [1, 4]
    finally:
        release.set()
        worker.close()


def test_worker_error_does_not_disable_subsequent_frames():
    def decode(value):
        if value == 1:
            raise ValueError('bad image')
        return value
    worker = LatestWorker(decode)
    try:
        worker.put(1)
        worker.start()
        with pytest.raises(ValueError):
            worker.running[1].result(timeout=3)
        item, output, error = worker.take_done()
        assert item == 1 and output is None and str(error) == 'bad image'
        worker.put(2)
        assert worker.start()
        worker.running[1].result(timeout=3)
        assert worker.take_done() == (2, 2, None)
    finally:
        worker.close()
