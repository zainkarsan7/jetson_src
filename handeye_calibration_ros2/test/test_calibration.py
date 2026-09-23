import numpy as np
import pytest

from handeye_calibration_ros2.calibration import Sample, check_capture, evaluate, solve, METHODS
from handeye_calibration_ros2.geometry import distance, from_quaternion, inverse, quaternion, rigid, transform


def dataset(mode='eye_in_hand', count=25, noise=False, seed=4):
    rng = np.random.default_rng(seed)
    x = transform([0.3, -0.2, 0.4], [0.08, -0.04, 0.12])
    constant = transform([-0.2, 0.4, 0.1], [0.4, 0.1, 0.7])
    samples = []
    for i in range(count):
        bg = transform(rng.uniform(-0.8, 0.8, 3), rng.uniform(-0.4, 0.4, 3))
        ct = inverse(x) @ (inverse(bg) if mode == 'eye_in_hand' else bg) @ constant
        if noise:
            ct = ct @ transform(rng.normal(0, 0.0005, 3), rng.normal(0, 0.0002, 3))
        samples.append(Sample(bg, ct, i*1000000000, 0.2))
    return samples, x, constant


@pytest.mark.parametrize('mode', ['eye_in_hand', 'eye_to_hand'])
@pytest.mark.parametrize('method', list(METHODS))
def test_all_solver_conventions_against_known_transform(mode, method):
    samples, expected, _ = dataset(mode)
    result = solve(samples, mode, method)
    actual = np.asarray(result['parent_from_camera'])
    assert np.allclose(actual, expected, atol=1e-7)
    assert result['residuals']['translation_rms_m'] < 1e-8
    assert result['residuals']['rotation_rms_deg'] < 1e-4


@pytest.mark.parametrize('mode', ['eye_in_hand', 'eye_to_hand'])
def test_noisy_samples_and_held_out_validation(mode):
    samples, expected, constant = dataset(mode, noise=True)
    result = solve(samples[:18], mode)
    actual = np.asarray(result['parent_from_camera'])
    translation, rotation = distance(actual, expected)
    assert translation < 0.002
    assert rotation < 0.005
    validation = evaluate(samples[18:], actual, mode, result['residuals']['closure_reference'])
    assert validation['translation_rms_m'] < 0.003
    assert validation['rotation_rms_deg'] < 0.3
    wrong = evaluate(samples[18:], inverse(actual), mode, constant)
    assert wrong['translation_rms_m'] > 0.05


def test_unobservable_motion_rejected():
    samples = [Sample(transform([0, 0, a]), transform()) for a in np.linspace(0, 1, 8)]
    with pytest.raises(ValueError, match='diversity'):
        solve(samples)
    with pytest.raises(ValueError, match='at least'):
        solve(samples[:4])


def test_invalid_transforms_and_quaternions():
    for bad in [np.zeros((4, 4)), np.full((4, 4), np.nan), np.diag([1, 1, -1, 1])]:
        with pytest.raises(ValueError):
            rigid(bad)
    with pytest.raises(ValueError):
        from_quaternion([0, 0, 0], [0, 0, 0, 0])
    for v in ([0, 0, 0], [0.3, -0.5, 0.8], [np.pi, 0, 0]):
        t = transform(v, [0.1, 0.2, 0.3])
        assert np.allclose(from_quaternion(t[:3, 3], quaternion(t)), t, atol=1e-7)


def stable_history():
    return [Sample(transform(), transform(translation=[0, 0, 1]), i*100000000, 0.2)
            for i in range(10, 18)]


def test_capture_requires_fresh_distinct_settled_data():
    history = stable_history()
    assert check_capture(history, [], 1800000000) is history[-1]
    with pytest.raises(ValueError, match='stale'):
        check_capture(history, [], 5000000000)
    with pytest.raises(ValueError, match='Duplicate'):
        check_capture(history, [history[-1]], 1800000000)
    history[-2].base_from_effector = transform(translation=[0.02, 0, 0])
    with pytest.raises(ValueError, match='moving'):
        check_capture(history, [], 1800000000)


def test_observation_gap_cannot_count_as_settling():
    history = stable_history()
    with pytest.raises(ValueError, match='continuous'):
        check_capture([history[0], history[-2], history[-1]], [], 1800000000, max_gap_s=0.25)
