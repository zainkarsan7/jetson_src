"""ArUco/ChArUco targets using the OpenCV 4.5/4.6 APIs shipped with Jammy."""
from dataclasses import asdict, dataclass

import cv2
import numpy as np

from .geometry import angle, transform


@dataclass(frozen=True)
class BoardSpec:
    kind: str = 'charuco'
    dictionary: str = 'DICT_5X5_250'
    columns: int = 5
    rows: int = 7
    square_length_m: float = 0.04
    marker_length_m: float = 0.03
    marker_separation_m: float = 0.01

    def __post_init__(self):
        if self.kind not in ('charuco', 'aruco'):
            raise ValueError('board kind must be charuco or aruco')
        if not isinstance(self.columns, int) or not isinstance(self.rows, int):
            raise ValueError('Board dimensions must be integers')
        if not 2 <= self.columns <= 30 or not 2 <= self.rows <= 30:
            raise ValueError('Board rows/columns must be between 2 and 30')
        lengths = [self.square_length_m, self.marker_length_m, self.marker_separation_m]
        if not np.isfinite(lengths).all() or min(lengths) <= 0:
            raise ValueError('Board lengths must be positive, finite metres')
        if self.kind == 'charuco' and self.marker_length_m >= self.square_length_m:
            raise ValueError('ChArUco marker must be smaller than its square')
        if not self.dictionary.startswith('DICT_') or not hasattr(cv2.aruco, self.dictionary):
            raise ValueError('Unknown ArUco dictionary')
        capacity = len(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)).bytesList)
        count = self.columns*self.rows if self.kind == 'aruco' else self.columns*self.rows//2
        if count > capacity:
            raise ValueError('Board requires more marker IDs than the dictionary provides')

    def to_dict(self):
        return asdict(self)

    def dimensions(self):
        if self.kind == 'charuco':
            return self.columns*self.square_length_m, self.rows*self.square_length_m
        return (self.columns*self.marker_length_m+(self.columns-1)*self.marker_separation_m,
                self.rows*self.marker_length_m+(self.rows-1)*self.marker_separation_m)


class Target:
    def __init__(self, spec):
        self.spec = spec
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
        if spec.kind == 'charuco':
            if hasattr(cv2.aruco, 'CharucoBoard_create'):
                self.board = cv2.aruco.CharucoBoard_create(
                    spec.columns, spec.rows, spec.square_length_m, spec.marker_length_m, self.dictionary)
            else:
                self.board = cv2.aruco.CharucoBoard(
                    (spec.columns, spec.rows), spec.square_length_m, spec.marker_length_m, self.dictionary)
                # Match the pre-4.6 printed pattern, including even row counts.
                if hasattr(self.board, 'setLegacyPattern'):
                    self.board.setLegacyPattern(True)
        else:
            if hasattr(cv2.aruco, 'GridBoard_create'):
                self.board = cv2.aruco.GridBoard_create(
                    spec.columns, spec.rows, spec.marker_length_m, spec.marker_separation_m, self.dictionary)
            else:
                self.board = cv2.aruco.GridBoard(
                    (spec.columns, spec.rows), spec.marker_length_m, spec.marker_separation_m, self.dictionary)
        self.parameters = (cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create')
                           else cv2.aruco.DetectorParameters())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def render(self, pixels_per_metre=5000, margin_px=40):
        if not 500 <= pixels_per_metre <= 20000 or not 10 <= margin_px <= 1000:
            raise ValueError('Use 500..20000 pixels/metre and 10..1000 pixel margins')
        w, h = self.spec.dimensions()
        size = (round(w*pixels_per_metre)+2*margin_px, round(h*pixels_per_metre)+2*margin_px)
        if max(size) > 12000:
            raise ValueError('Target image too large; reduce pixels/metre')
        if hasattr(self.board, 'draw'):
            return self.board.draw(size, marginSize=margin_px, borderBits=1)
        return self.board.generateImage(size, marginSize=margin_px, borderBits=1)

    def correspondences(self, gray, k, d):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is None or len(ids) < 2:
            raise ValueError('Need at least two visible board markers')
        if self.spec.kind == 'charuco':
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board, cameraMatrix=k, distCoeffs=d)
            if ci is None or len(ci) < 6:
                raise ValueError('Need at least six ChArUco corners')
            pts = (self.board.chessboardCorners if hasattr(self.board, 'chessboardCorners')
                   else self.board.getChessboardCorners())
            obj, img = pts[ci.flatten()], cc.reshape(-1, 2)
        else:
            board_ids = self.board.ids if hasattr(self.board, 'ids') else self.board.getIds()
            board_pts = self.board.objPoints if hasattr(self.board, 'objPoints') else self.board.getObjPoints()
            lookup = {int(i): p for i, p in zip(np.asarray(board_ids).flatten(), board_pts)}
            pairs = [(lookup[int(i)], c.reshape(4, 2)) for i, c in zip(ids.flatten(), corners) if int(i) in lookup]
            if len(pairs) < 2:
                raise ValueError('Detected markers do not belong to this board')
            obj, img = np.concatenate([p[0] for p in pairs]), np.concatenate([p[1] for p in pairs])
        if np.linalg.matrix_rank(obj[:, :2] - np.mean(obj[:, :2], axis=0), tol=1e-5) < 2:
            raise ValueError('Visible board corners are collinear')
        return np.ascontiguousarray(obj, dtype=np.float64), np.ascontiguousarray(img, dtype=np.float64)

    def detect(self, bgr, k, d, max_reprojection_px=2.0):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        obj, img = self.correspondences(gray, k, d)
        ok, rotations, translations, _ = cv2.solvePnPGeneric(obj, img, k, d, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise ValueError('Target pose estimation failed')
        candidates = []
        for r, t in zip(rotations, translations):
            pose = transform(r, t)
            if np.min((pose[:3, :3] @ obj.T + pose[:3, 3:4])[2]) <= 0:
                continue
            projected = cv2.projectPoints(obj, r, t, k, d)[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean(np.sum((projected-img)**2, axis=1))))
            candidates.append((error, pose, r, t))
        if not candidates:
            raise ValueError('Board pose is behind the camera')
        candidates.sort(key=lambda x: x[0])
        error, pose, r, t = candidates[0]
        if len(candidates) > 1:
            second = candidates[1]
            if (second[0]-error < 0.1 and
                    angle(pose[:3, :3].T @ second[1][:3, :3]) > np.deg2rad(5)):
                raise ValueError('Ambiguous planar pose: tilt the board/camera or move closer')
        if error > max_reprojection_px:
            raise ValueError(f'Target reprojection RMS {error:.2f}px exceeds {max_reprojection_px:.2f}px')
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if bgr.ndim == 2 else bgr.copy()
        for p in img:
            cv2.circle(debug, tuple(np.round(p).astype(int)), 3, (0, 255, 0), -1)
        cv2.drawFrameAxes(debug, k, d, r, t, self.spec.marker_length_m)
        cv2.putText(debug, f'{len(img)} corners; RMS {error:.2f}px', (12, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return pose, error, debug


def camera_model(info, image_width, image_height, rectified=False):
    """Return K,D and physical-optical-from-rectified-optical rotation."""
    if info.width != image_width or info.height != image_height:
        raise ValueError('Image and CameraInfo resolution mismatch')
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('ROI/binning unsupported: provide full-resolution images and matching CameraInfo')
    if info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height):
        raise ValueError('Cropped ROI unsupported')
    correction = np.eye(3)
    if rectified:
        p = np.asarray(info.p, dtype=float).reshape(3, 4)
        if not np.allclose(p[:, 3], 0):
            raise ValueError('Stereo translated projection unsupported; use the raw camera stream')
        k, d = p[:, :3].copy(), np.zeros(5)
        correction = np.asarray(info.r, dtype=float).reshape(3, 3).T
        transform(correction)
    else:
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('Raw stream requires plumb_bob/rational_polynomial CameraInfo')
        k, d = np.asarray(info.k, dtype=float).reshape(3, 3), np.asarray(info.d, dtype=float)
        if len(d) not in (4, 5, 8, 12, 14):
            raise ValueError('Unsupported distortion coefficient count')
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('CameraInfo contains invalid or uncalibrated intrinsics')
    return k, d, correction
