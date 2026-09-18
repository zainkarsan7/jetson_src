from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from handeye_calibration_ros2.geometry import distance, transform
from handeye_calibration_ros2.target import BoardSpec, Target, camera_model


def render_observation(spec):
    target = Target(spec)
    ppm, margin = 5000, 40
    board = target.render(ppm, margin)
    w, h = spec.dimensions()
    k = np.array([[950., 0, 640], [0, 950, 480], [0, 0, 1]])
    rvec, tvec = np.array([0.25, -0.15, 0.03]), np.array([-w/2, -h/2, 0.7])
    obj = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=float)
    dst = cv2.projectPoints(obj, rvec, tvec, k, np.zeros(5))[0].reshape(-1, 2).astype(np.float32)
    src = np.float32([[margin, margin], [board.shape[1]-margin, margin],
                      [board.shape[1]-margin, board.shape[0]-margin], [margin, board.shape[0]-margin]])
    image = cv2.warpPerspective(board, cv2.getPerspectiveTransform(src, dst), (1280, 960), borderValue=255)
    expected = transform(rvec, tvec)
    # Before 4.6 OpenCV's board origin is bottom-left, y-up, z-out;
    # newer boards use top-left, y-down, z-in. This is the target frame,
    # not the camera optical convention, and is eliminated by hand-eye solving.
    if tuple(map(int, cv2.__version__.split('.')[:2])) < (4, 6):
        expected = expected @ transform([np.pi, 0, 0], [0, h, 0])
    return target, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), k, expected


@pytest.mark.parametrize('kind', ['charuco', 'aruco'])
def test_generated_target_detection_roundtrip(kind):
    target, image, k, expected = render_observation(BoardSpec(kind=kind))
    actual, rms, debug = target.detect(image, k, np.zeros(5))
    translation, rotation = distance(actual, expected)
    assert translation < 0.005
    assert rotation < np.deg2rad(2)
    assert rms < 1.0
    assert debug.shape == image.shape


def test_no_target_rejected():
    with pytest.raises(ValueError, match='visible'):
        Target(BoardSpec()).detect(np.full((480, 640, 3), 255, np.uint8), np.eye(3), np.zeros(5))


def test_invalid_board_geometry():
    with pytest.raises(ValueError):
        BoardSpec(marker_length_m=0.05)
    with pytest.raises(ValueError):
        BoardSpec(dictionary='DICT_4X4_50', columns=30, rows=30)


def info():
    return SimpleNamespace(width=640, height=480, binning_x=0, binning_y=0,
                           roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0),
                           k=[500., 0, 320, 0, 500., 240, 0, 0, 1],
                           p=[510., 0, 320, 0, 0, 510., 240, 0, 0, 0, 1, 0],
                           r=np.eye(3).reshape(-1), d=[0.1, 0., 0., 0., 0.], distortion_model='plumb_bob')


def test_raw_and_rectified_models_are_not_mixed():
    i = info()
    k, d, correction = camera_model(i, 640, 480)
    assert k[0, 0] == 500 and d[0] == 0.1
    k, d, correction = camera_model(i, 640, 480, rectified=True)
    assert k[0, 0] == 510 and np.all(d == 0)
    i.r = transform([0.1, 0, 0])[:3, :3].reshape(-1)
    assert np.allclose(camera_model(i, 640, 480, True)[2], np.asarray(i.r).reshape(3, 3).T)
    with pytest.raises(ValueError, match='resolution'):
        camera_model(i, 320, 240)
    i.distortion_model = 'equidistant'
    with pytest.raises(ValueError, match='plumb_bob'):
        camera_model(i, 640, 480)
