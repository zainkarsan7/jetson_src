"""Standalone Qt GUI. ROS callbacks are pumped by Qt, so widgets stay on the UI thread."""
import argparse
import json
from pathlib import Path
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from .cli import generate_target
from .target import BoardSpec


class CalibrationWindow(QtWidgets.QMainWindow):
    def __init__(self, invoke):
        super().__init__()
        self.invoke = invoke
        self.state = None
        self.setWindowTitle('Hand–Eye Calibration · ROS 2 Humble')
        self.resize(1100, 800)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        self.context = QtWidgets.QLabel('Waiting for /handeye/status… Start calibration.launch.py first.')
        self.context.setWordWrap(True)
        layout.addWidget(self.context)
        self.instructions = QtWidgets.QLabel(
            'Move with MoveIt or the robot pendant. Keep the target visible, stop, then capture. '
            'Collect 15–25 varied poses with rotations about multiple axes.')
        self.instructions.setWordWrap(True)
        layout.addWidget(self.instructions)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(splitter, 1)
        self.image = QtWidgets.QLabel('Waiting for target detection image')
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setMinimumSize(480, 320)
        self.image.setStyleSheet('background: #18212c; color: white;')
        splitter.addWidget(self.image)
        right = QtWidgets.QWidget()
        right.setMinimumWidth(400)
        right_layout = QtWidgets.QVBoxLayout(right)
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(['Sample', 'Time (s)', 'RMS (px)'])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        right_layout.addWidget(self.table)
        self.result = QtWidgets.QPlainTextEdit()
        self.result.setReadOnly(True)
        self.result.setPlaceholderText('Calibration transform and closure residuals appear after Solve.')
        right_layout.addWidget(self.result)
        splitter.addWidget(right)
        splitter.setSizes([650, 430])
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        self.buttons = {}
        for action, label in [('capture', 'Capture sample'), ('undo', 'Undo last'), ('clear', 'Clear'),
                              ('load', 'Load session'), ('solve', 'Solve'), ('save', 'Export calibration')]:
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, a=action: self.request(a))
            controls.addWidget(button)
            self.buttons[action] = button
        self.target_button = QtWidgets.QPushButton('Save target PNG…')
        self.target_button.clicked.connect(self.save_target)
        controls.addWidget(self.target_button)
        self.readiness = QtWidgets.QLabel('Capture unavailable')
        self.readiness.setWordWrap(True)
        layout.addWidget(self.readiness)
        self.operation = QtWidgets.QLabel('')
        self.operation.setWordWrap(True)
        layout.addWidget(self.operation)
        self.statusBar().showMessage('Not connected')
        self.last_pixmap = None
        self.offline()

    def offline(self):
        for button in self.buttons.values():
            button.setEnabled(False)
        self.target_button.setEnabled(self.state is not None)
        self.statusBar().showMessage('No recent backend status')

    def request(self, action):
        if action == 'clear' and QtWidgets.QMessageBox.question(
                self, 'Clear session?', 'Clear all collected samples and overwrite the saved session?',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
            return
        self.invoke(action)

    def receive_state(self, state):
        self.state = state
        c = state['context']
        parent = c['effector_frame'] if c['mode'] == 'eye_in_hand' else c['base_frame']
        self.context.setText(f'{c["mode"].replace("_", " ").title()} | Robot: {c["base_frame"]} → '
                             f'{c["effector_frame"]} | Solve: {parent} ← {c["camera_frame"]}\n'
                             f'Image: {c["image_topic"]} | Output: {state["output_directory"]}')
        samples = state.get('samples', [])
        self.table.setRowCount(len(samples))
        for row, s in enumerate(samples):
            for col, text in enumerate([str(row+1), f'{s["stamp_ns"]/1e9:.3f}', f'{s["reprojection_error_px"]:.3f}']):
                self.table.setItem(row, col, QtWidgets.QTableWidgetItem(text))
        self.buttons['capture'].setEnabled(state['capture_ready'])
        self.buttons['undo'].setEnabled(bool(samples))
        self.buttons['clear'].setEnabled(bool(samples))
        self.buttons['load'].setEnabled(True)
        self.buttons['solve'].setEnabled(len(samples) >= state.get('min_samples', 5))
        result = state.get('result')
        self.buttons['save'].setEnabled(bool(result and result.get('quality_passed')))
        self.target_button.setEnabled(True)
        self.readiness.setText('Ready to capture' if state['capture_ready'] else state['capture_reason'])
        self.readiness.setStyleSheet('color: #167040;' if state['capture_ready'] else 'color: #975514;')
        timing = state.get('timing', {})
        fields = [('image_age_at_receive_s', 'Image age at receive'),
                  ('detection_duration_s', 'Detection'), ('paired_observation_age_s', 'Paired age')]
        details = ' | '.join(f'{label}: {timing[key]:.3f}s' for key, label in fields if key in timing)
        if details:
            self.readiness.setText(self.readiness.text()+'\n'+details)
        if result:
            res = result['residuals']
            text = (f'Parent: {result["parent_frame"]}\nCamera: {result["camera_frame"]}\n'
                    f'xyz (m): {", ".join(f"{v:.6f}" for v in result["translation_m"])}\n'
                    f'xyzw: {", ".join(f"{v:.6f}" for v in result["quaternion_xyzw"])}\n\n'
                    f'Closure RMS: {res["translation_rms_m"]*1000:.3f} mm, {res["rotation_rms_deg"]:.3f} deg\n'
                    f'Closure max: {res["translation_max_m"]*1000:.3f} mm, {res["rotation_max_deg"]:.3f} deg\n'
                    f'Quality: {"PASS" if result["quality_passed"] else "FAIL"}\n'
                    'Closure consistency is not an absolute accuracy guarantee.\n\n'
                    + '\n'.join(f'Sample {r["index"]+1}: {r["translation_m"]*1000:.2f} mm / '
                                f'{r["rotation_deg"]:.3f} deg' for r in res['per_sample']))
            if self.result.toPlainText() != text:
                self.result.setPlainText(text)
        else:
            self.result.clear()
        self.statusBar().showMessage(state['message'])

    def receive_image(self, bgr):
        rgb = bgr[:, :, ::-1].copy()
        image = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.last_pixmap = QtGui.QPixmap.fromImage(image)
        self.show_pixmap()

    def show_pixmap(self):
        if self.last_pixmap is not None:
            self.image.setPixmap(self.last_pixmap.scaled(self.image.size(), QtCore.Qt.KeepAspectRatio,
                                                       QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'last_pixmap'):
            self.show_pixmap()

    def save_target(self):
        if self.state is None:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, 'Save configured target', 'handeye_target.png', 'PNG (*.png)')
        if path:
            try:
                info = generate_target(path, BoardSpec(**self.state['context']['board']))
                self.operation.setText(f'Saved {path}; board region {info["board_width_m"]*1000:.1f} × '
                                       f'{info["board_height_m"]*1000:.1f} mm. Measure print before use.')
            except (ValueError, OSError) as exc:
                QtWidgets.QMessageBox.warning(self, 'Target generation failed', str(exc))


def main(argv=None):
    # Qt is imported independently of ROS so its layout can also be exercised off-robot.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import remove_ros_args
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', default='/handeye', help='Fully qualified calibration node name')
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    app = QtWidgets.QApplication([argv[0]])
    rclpy.init(args=argv)
    node = Node('handeye_gui')
    bridge = CvBridge()
    backend = options.backend.rstrip('/')
    clients = {name: node.create_client(Trigger, backend+'/'+name)
               for name in ('capture', 'undo', 'clear', 'load', 'solve', 'save')}
    pending = []
    last_status = [0.0]

    def invoke(action):
        if pending:
            window.operation.setText('An operation is already pending')
            return
        if not clients[action].service_is_ready():
            window.operation.setText('Service unavailable: '+backend+'/'+action)
            return
        pending.append((clients[action].call_async(Trigger.Request()), time.monotonic()))
        window.operation.setText(action.title()+' requested…')

    window = CalibrationWindow(invoke)

    def on_status(msg):
        try:
            window.receive_state(json.loads(msg.data))
            last_status[0] = time.monotonic()
        except (ValueError, KeyError, TypeError) as exc:
            window.operation.setText('Invalid backend status: '+str(exc))

    def on_image(msg):
        try:
            window.receive_image(bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        except Exception as exc:
            window.operation.setText('Image display failed: '+str(exc))

    subscriptions = [node.create_subscription(String, backend+'/status', on_status, 10),
                     node.create_subscription(Image, backend+'/target_detection', on_image, qos_profile_sensor_data)]

    def tick():
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.0)
        if pending:
            future, started = pending[0]
            if future.done():
                try:
                    response = future.result()
                    window.operation.setText(('OK: ' if response.success else 'Failed: ')+response.message)
                except Exception as exc:
                    window.operation.setText('Service failed: '+str(exc))
                pending.clear()
            elif time.monotonic()-started > 30:
                window.operation.setText('Backend response delayed; operation may still complete.')
        if time.monotonic()-last_status[0] > 2:
            window.offline()

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(20)
    window.show()
    try:
        app.exec_()
    finally:
        timer.stop()
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
