"""Versioned, auditable sessions and ROS 2 transform export."""
from pathlib import Path
import os
import tempfile
from xml.sax.saxutils import quoteattr

import numpy as np
import yaml

from .calibration import MODES, Sample
from .geometry import quaternion, rigid


def atomic_text(path, text):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_yaml(path, data):
    atomic_text(path, yaml.safe_dump(data, sort_keys=False))


def read_yaml(path):
    try:
        with Path(path).expanduser().open(encoding='utf-8') as f:
            result = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise ValueError('Malformed calibration YAML') from exc
    if not isinstance(result, dict) or result.get('schema_version') != 1:
        raise ValueError('Not a schema_version: 1 calibration file')
    return result


def save_session(path, context, samples):
    write_yaml(path, {'schema_version': 1, 'context': context,
                      'samples': [s.to_dict() for s in samples]})


def load_session(path, expected_context=None):
    data = read_yaml(path)
    context = data.get('context', {})
    if context.get('mode') not in MODES:
        raise ValueError('Invalid session mode')
    for key in ('base_frame', 'effector_frame', 'camera_frame'):
        if not isinstance(context.get(key), str) or not context[key]:
            raise ValueError(f'Missing session {key}')
    if expected_context is not None and context != expected_context:
        raise ValueError('Session context differs from current frames, board, or camera configuration')
    entries = data.get('samples')
    if not isinstance(entries, list) or len(entries) > 500:
        raise ValueError('Expected at most 500 samples')
    return context, [Sample(**s) for s in entries]


def complete_result(result, context):
    result = dict(result)
    result['context'] = context
    result['parent_frame'] = context['effector_frame'] if result['mode'] == 'eye_in_hand' else context['base_frame']
    result['camera_frame'] = context['camera_frame']
    result['opencv_version'] = __import__('cv2').__version__
    return result


def export_result(directory, result):
    directory = Path(directory).expanduser()
    t = rigid(result['parent_from_camera'])
    parent, child = result['parent_frame'], result['camera_frame']
    args = ['--x', str(t[0, 3]), '--y', str(t[1, 3]), '--z', str(t[2, 3])]
    for key, value in zip(('--qx', '--qy', '--qz', '--qw'), quaternion(t)):
        args += [key, str(value)]
    args += ['--frame-id', parent]
    # A new child name avoids competing with the live camera's existing TF parent.
    launch = '''"""Calibration preview. Integrate the physical mount into your URDF separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('child_frame', default_value=CHILD),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='handeye_calibrated_camera',
             arguments=ARGS + ['--child-frame-id', LaunchConfiguration('child_frame')]),
    ])
'''.replace('CHILD', repr(child+'_calibrated')).replace('ARGS', repr(args))
    atomic_text(directory/'calibration.launch.py', launch)
    # The optional mount transform uses only known intra-camera rigid transforms.
    if 'parent_from_mount' in result:
        mount = rigid(result['parent_from_mount'])
        r = mount[:3, :3]
        pitch = float(np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0])))
        if abs(np.cos(pitch)) > 1e-8:
            roll, yaw = np.arctan2(r[2, 1], r[2, 2]), np.arctan2(r[1, 0], r[0, 0])
        else:
            roll, yaw = np.arctan2(-r[1, 2], r[1, 1]), 0.0
        xyz = ' '.join(map(str, mount[:3, 3]))
        rpy = ' '.join(map(str, (roll, pitch, yaw)))
        xml = ('<!-- Replace the existing mount joint; do not add a second parent. -->\n'
               '<joint name="calibrated_camera_mount" type="fixed">\n'
               f'  <parent link={quoteattr(parent)}/>\n'
               f'  <child link={quoteattr(result["mount_frame"])}/>\n'
               f'  <origin xyz="{xyz}" rpy="{rpy}"/>\n</joint>\n')
        atomic_text(directory/'mount_joint.xml', xml)
    write_yaml(directory/'calibration.yaml', result)
