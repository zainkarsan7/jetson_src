"""Hand-eye solver and explicit frame conventions; no ROS imports."""
from dataclasses import dataclass
import itertools

import cv2
import numpy as np

from .geometry import average, distance, inverse, quaternion, rigid, transform
from .timing import check_age

METHODS = {
    'park': cv2.CALIB_HAND_EYE_PARK,
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MODES = ('eye_in_hand', 'eye_to_hand')


@dataclass
class Sample:
    base_from_effector: np.ndarray
    camera_from_target: np.ndarray
    stamp_ns: int = 0
    reprojection_error_px: float = 0.0

    def __post_init__(self):
        self.base_from_effector = rigid(self.base_from_effector)
        self.camera_from_target = rigid(self.camera_from_target)
        if self.stamp_ns < 0 or not np.isfinite(self.reprojection_error_px) or self.reprojection_error_px < 0:
            raise ValueError('Invalid sample timestamp or reprojection error')

    def to_dict(self):
        return dict(base_from_effector=self.base_from_effector.tolist(),
                    camera_from_target=self.camera_from_target.tolist(),
                    stamp_ns=int(self.stamp_ns),
                    reprojection_error_px=float(self.reprojection_error_px))


def excitation(samples):
    """Reject absent/single-axis rotation; report a heuristic, not covariance."""
    vectors = []
    constraints = []
    for a, b in itertools.combinations(samples, 2):
        r = a.base_from_effector[:3, :3].T @ b.base_from_effector[:3, :3]
        vectors.append(cv2.Rodrigues(r)[0].reshape(3))
        constraints.append(r - np.eye(3))
    sv = np.linalg.svd(np.array(vectors), compute_uv=False)
    tv = np.linalg.svd(np.vstack(constraints), compute_uv=False)
    max_angle = float(np.max(np.linalg.norm(vectors, axis=1)))
    if max_angle < np.deg2rad(10) or sv[1] < 0.05 * sv[0] or tv[-1] < 0.01:
        raise ValueError('Insufficient rotational diversity: use appreciable rotations about at least two axes')
    return {'rotation_singular_values': sv.tolist(),
            'translation_constraint_singular_values': tv.tolist(),
            'maximum_relative_rotation_deg': float(np.rad2deg(max_angle))}


def closure_transforms(samples, x, mode):
    if mode not in MODES:
        raise ValueError('mode must be eye_in_hand or eye_to_hand')
    # Eye in hand: B_T_G G_T_C C_T_T = constant B_T_T.
    # Eye to hand: G_T_B B_T_C C_T_T = constant G_T_T.
    return [(s.base_from_effector if mode == 'eye_in_hand' else inverse(s.base_from_effector))
            @ x @ s.camera_from_target for s in samples]


def evaluate(samples, x, mode, reference=None):
    if not samples:
        raise ValueError('No samples to evaluate')
    closure = closure_transforms(samples, rigid(x), mode)
    ref = average(closure) if reference is None else rigid(reference)
    errors = np.asarray([distance(ref, t) for t in closure])
    return {
        'closure_reference': ref.tolist(),
        'translation_rms_m': float(np.sqrt(np.mean(errors[:, 0]**2))),
        'rotation_rms_deg': float(np.rad2deg(np.sqrt(np.mean(errors[:, 1]**2)))),
        'translation_max_m': float(np.max(errors[:, 0])),
        'rotation_max_deg': float(np.rad2deg(np.max(errors[:, 1]))),
        'per_sample': [dict(index=i, translation_m=float(e[0]), rotation_deg=float(np.rad2deg(e[1])))
                       for i, e in enumerate(errors)],
    }


def solve(samples, mode='eye_in_hand', method='park', min_samples=5):
    if mode not in MODES or method not in METHODS:
        raise ValueError('Unknown calibration mode or solver method')
    if len(samples) < max(5, min_samples):
        raise ValueError(f'Need at least {max(5, min_samples)} samples; have {len(samples)}')
    quality = excitation(samples)
    a = [s.base_from_effector if mode == 'eye_in_hand' else inverse(s.base_from_effector)
         for s in samples]
    c = [s.camera_from_target for s in samples]
    r, t = cv2.calibrateHandEye(
        [v[:3, :3] for v in a], [v[:3, 3] for v in a],
        [v[:3, :3] for v in c], [v[:3, 3] for v in c], method=METHODS[method])
    x = transform(r, t)  # also rejects NaNs/invalid solver output
    return {'schema_version': 1, 'mode': mode, 'method': method, 'sample_count': len(samples),
            'parent_from_camera': x.tolist(), 'translation_m': x[:3, 3].tolist(),
            'quaternion_xyzw': quaternion(x), 'excitation': quality,
            'residuals': evaluate(samples, x, mode)}


def check_capture(history, samples, now_ns, max_age_s=1.0, settle_s=0.5,
                  stability_translation_m=0.002, stability_rotation_deg=0.5,
                  duplicate_translation_m=0.005, duplicate_rotation_deg=3.0,
                  max_gap_s=1.0):
    """Require a fresh, uninterrupted, stable interval and a distinct robot pose."""
    if not history:
        raise ValueError('No valid target observation paired with timestamped robot TF')
    latest = history[-1]
    check_age(latest.stamp_ns, now_ns, max_age_s, 'Latest paired observation')
    cutoff = latest.stamp_ns - int(settle_s * 1e9)
    earlier = [i for i, s in enumerate(history) if s.stamp_ns <= cutoff]
    if not earlier:
        raise ValueError('Wait for the robot and target to settle')
    # At low detector rates, include enough older samples to have at least three.
    window = list(history)[min(earlier[-1], max(0, len(history)-3)):]
    if len(window) < 3:
        raise ValueError(f'Need at least 3 settled observations; have {len(window)}')
    gaps = [(b.stamp_ns-a.stamp_ns)/1e9 for a, b in zip(window, window[1:])]
    if min(gaps) <= 0 or max(gaps) > max_gap_s:
        raise ValueError(f'Insufficient continuous observations: largest gap {max(gaps):.3f}s; '
                         f'limit {max_gap_s:.3f}s (max_observation_gap_s)')
    for s in window:
        for attr in ('base_from_effector', 'camera_from_target'):
            d, a = distance(getattr(s, attr), getattr(latest, attr))
            if d > stability_translation_m or a > np.deg2rad(stability_rotation_deg):
                raise ValueError('Robot or observed target is still moving/noisy')
    for s in samples:
        d, a = distance(s.base_from_effector, latest.base_from_effector)
        if d < duplicate_translation_m and a < np.deg2rad(duplicate_rotation_deg):
            raise ValueError('Duplicate robot pose: move and rotate before sampling again')
    return latest
