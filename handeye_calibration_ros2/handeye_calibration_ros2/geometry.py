"""T_A_B maps coordinates in B into A. Distances: metres; angles: radians."""
import cv2
import numpy as np


def rigid(value):
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError('Expected a finite 4x4 transform')
    r = t[:3, :3]
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError('Invalid homogeneous bottom row')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r)-1) > 1e-5:
        raise ValueError('Rotation must be in SO(3)')
    return t.copy()


def transform(rotation=None, translation=None):
    t = np.eye(4)
    if rotation is not None:
        a = np.asarray(rotation, dtype=float)
        t[:3, :3] = cv2.Rodrigues(a.reshape(3))[0] if a.size == 3 else a
    if translation is not None:
        t[:3, 3] = np.asarray(translation).reshape(3)
    return rigid(t)


def inverse(t):
    t = rigid(t)
    return transform(t[:3, :3].T, -t[:3, :3].T @ t[:3, 3])


def angle(rotation):
    return float(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))


def distance(a, b):
    return float(np.linalg.norm(a[:3, 3]-b[:3, 3])), angle(a[:3, :3].T @ b[:3, :3])


def from_quaternion(xyz, xyzw):
    q = np.asarray(xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    r = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return transform(r, xyz)


def quaternion(t):
    v = cv2.Rodrigues(rigid(t)[:3, :3])[0].reshape(3)
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [*map(float, v/theta*np.sin(theta/2)), float(np.cos(theta/2))]


def average(transforms):
    u, _, vt = np.linalg.svd(sum(t[:3, :3] for t in transforms))
    r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return transform(r, np.mean([t[:3, 3] for t in transforms], axis=0))
