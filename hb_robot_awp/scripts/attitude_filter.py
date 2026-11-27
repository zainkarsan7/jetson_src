"""Small quaternion complementary estimator. No ROS, no authority, no prediction on gaps."""
from collections import deque
import math
import numpy as np


def multiply(a, b):  # w,x,y,z; body -> level
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def conjugate(q):
    return q * np.array([1., -1., -1., -1.])


def rotate(q, v):
    return multiply(multiply(q, np.r_[0., v]), conjugate(q))[1:]


def rpy(q):
    w, x, y, z = q
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(np.clip(2*(w*y-z*x), -1, 1)),
                     math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])


class AttitudeFilter:
    def __init__(self, gain=1.0, calibration_s=2.0, max_dt=0.15):
        if not (0 < gain <= 10 and 1 <= calibration_s <= 30 and 0.01 <= max_dt <= 0.5):
            raise ValueError('Invalid filter parameters')
        self.gain, self.calibration_s, self.max_dt = gain, calibration_s, max_dt
        self.q = np.array([1., 0., 0., 0.])
        self.reference = self.q.copy()
        self.bias = np.zeros(3)
        self.window = deque()
        self.last = None
        self.calibration_generation = 0
        self.calibrated = False
        self.valid = False
        self.gravity_used = False
        self.dt = 0.
        self.status = 'Awaiting stationary bias calibration'

    def zero(self):
        if not self.valid:
            raise ValueError('Need a valid, fresh estimate to zero')
        self.reference = self.q.copy()

    def update(self, stamp, accel, gyro):
        a, g = np.asarray(accel, dtype=float), np.asarray(gyro, dtype=float)
        self.valid = False
        self.gravity_used = False
        if not (math.isfinite(stamp) and np.isfinite(a).all() and np.isfinite(g).all()):
            self.status = 'Non-finite sample'; return False
        if self.last is not None and stamp <= self.last:
            self.status = 'Duplicate or backward stamp'; return False
        self.dt = 0 if self.last is None else stamp-self.last
        self.last = stamp
        norm = np.linalg.norm(a)
        gravity_ok = abs(norm-9.80665) < 0.2*9.80665
        if not self.calibrated:
            if not gravity_ok or np.linalg.norm(g) > 0.2:
                self.window.clear(); self.status = 'Waiting for stationary calibration'; return False
            self.window.append((stamp, a, g))
            while self.window and stamp-self.window[0][0] > self.calibration_s+0.1:
                self.window.popleft()
            if len(self.window) < 30 or stamp-self.window[0][0] < self.calibration_s:
                return False
            aa = np.array([x[1] for x in self.window]); gg = np.array([x[2] for x in self.window])
            if np.linalg.norm(aa.std(axis=0)) > 0.15 or np.linalg.norm(gg.std(axis=0)) > 0.015:
                self.status = 'Calibration window not stationary'; return False
            self.bias = gg.mean(axis=0)
            ax, ay, az = aa.mean(axis=0)
            roll = math.atan2(ay, az); pitch = math.atan2(-ax, math.hypot(ay, az))
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            self.q = np.array([cr*cp, sr*cp, cr*sp, -sr*sp])
            self.reference = self.q.copy(); self.calibrated = True
            self.calibration_generation += 1
        elif not 0 < self.dt <= self.max_dt:
            # Do not integrate an unknown interval or silently reuse an old attitude.
            self.calibrated = False; self.window.clear()
            self.status = 'Sample gap: stationary recalibration required'; return False
        else:
            omega = g-self.bias
            if gravity_ok:
                predicted = rotate(conjugate(self.q), np.array([0., 0., 1.]))
                omega += self.gain*np.cross(a/norm, predicted)
                self.gravity_used = True
            self.q += 0.5*multiply(self.q, np.r_[0., omega])*self.dt
            self.q /= np.linalg.norm(self.q)
        self.valid = True
        self.status = 'Gravity-corrected' if self.gravity_used else 'Gyro only; acceleration rejected'
        return True

    def delta(self):
        return rpy(multiply(conjugate(self.reference), self.q))
