import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
pytest.importorskip('PyQt5')
from PyQt5 import QtWidgets
from handeye_calibration_ros2.gui import CalibrationWindow
from handeye_calibration_ros2.target import BoardSpec


def test_window_state_and_capture_action():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls = []
    window = CalibrationWindow(calls.append)
    state = {'context': {'mode': 'eye_in_hand', 'base_frame': 'base', 'effector_frame': 'tool',
                         'camera_frame': 'optical', 'image_topic': '/image', 'board': BoardSpec().to_dict()},
             'sample_count': 0, 'samples': [], 'capture_ready': True, 'capture_reason': '',
             'output_directory': '/tmp/calibration', 'result': None, 'message': 'Ready'}
    window.receive_state(state)
    window.buttons['capture'].click()
    assert calls == ['capture']
    assert not window.buttons['save'].isEnabled()
    window.offline()
    assert not window.buttons['capture'].isEnabled()
    window.close()
    app.processEvents()
