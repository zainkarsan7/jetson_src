import ast
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from handeye_calibration_ros2.calibration import solve
from handeye_calibration_ros2.geometry import transform
from handeye_calibration_ros2.storage import complete_result, export_result, load_session, read_yaml, save_session
from test_calibration import dataset


CONTEXT = {'mode': 'eye_in_hand', 'base_frame': 'base', 'effector_frame': 'tool', 'camera_frame': 'optical'}


def test_session_roundtrip_and_context_rejection(tmp_path):
    samples, _, _ = dataset()
    path = tmp_path/'session.yaml'
    save_session(path, CONTEXT, samples)
    context, loaded = load_session(path, CONTEXT)
    assert len(loaded) == len(samples)
    assert np.allclose(loaded[3].base_from_effector, samples[3].base_from_effector)
    with pytest.raises(ValueError, match='context'):
        load_session(path, dict(CONTEXT, effector_frame='wrong'))


def test_export_preview_and_mount_transform(tmp_path):
    samples, expected, _ = dataset()
    result = complete_result(solve(samples), CONTEXT)
    optical_from_mount = transform([0.2, 0.3, -0.1], [0.01, 0.02, 0])
    mount = expected @ optical_from_mount
    result.update(parent_from_mount=mount.tolist(), mount_frame='camera_base')
    export_result(tmp_path, result)
    ast.parse((tmp_path/'calibration.launch.py').read_text())
    assert 'optical_calibrated' in (tmp_path/'calibration.launch.py').read_text()
    saved = read_yaml(tmp_path/'calibration.yaml')
    assert np.allclose(saved['parent_from_camera'], expected)
    joint = ET.parse(tmp_path/'mount_joint.xml').getroot()
    assert joint.find('child').attrib['link'] == 'camera_base'
    assert np.allclose(np.fromstring(joint.find('origin').attrib['xyz'], sep=' '), mount[:3, 3])
    roll, pitch, yaw = np.fromstring(joint.find('origin').attrib['rpy'], sep=' ')
    rotation = transform([0, 0, yaw]) @ transform([0, pitch, 0]) @ transform([roll, 0, 0])
    assert np.allclose(rotation[:3, :3], mount[:3, :3])


def test_atomic_write_preserves_previous_file_on_failure(tmp_path, monkeypatch):
    import handeye_calibration_ros2.storage as storage
    path = tmp_path/'session.yaml'
    save_session(path, CONTEXT, [])
    original = path.read_bytes()

    def fail(*args):
        raise OSError('simulated write failure')

    monkeypatch.setattr(storage.os, 'replace', fail)
    with pytest.raises(OSError):
        save_session(path, CONTEXT, dataset()[0])
    assert path.read_bytes() == original
