#!/usr/bin/env python3
"""Read MCAP/SQLite bags without replaying any robot commands or authority topics."""
import argparse
import csv
from contextlib import ExitStack
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Bag directory with metadata.yaml')
    parser.add_argument('--output', default='experiment_csv')
    parser.add_argument('--storage', default='mcap', choices=['mcap', 'sqlite3'])
    args = parser.parse_args()
    folder = Path(args.output); folder.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {'hb_robot_interfaces/msg/SchedulingDiagnostics': 'timing',
              'hb_robot_awp/msg/TiltSample': 'tilt'}
    event_topics = {'/cut_pose_experiment/events', '/awp_excitation_server/events'}
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()
              if t.type in wanted or t.name in event_topics}
    if not topics: raise SystemExit('No experiment/timing topics in bag')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    types = {name: get_message(kind) for name, kind in topics.items()}
    writers = {}
    with ExitStack() as stack:
        while reader.has_next():
            topic, raw, bag_ns = reader.read_next(); m = deserialize_message(raw, types[topic])
            group = wanted.get(topics[topic], 'events')
            row = {'topic': topic, 'bag_time_ns': bag_ns}
            if group == 'events': row['event'] = m.data
            else:
                row['sample_stamp_ns'] = m.header.stamp.sec*10**9+m.header.stamp.nanosec
                for key in m.get_fields_and_field_types():
                    if key == 'header': continue
                    value = getattr(m, key)
                    if hasattr(value, 'x'):
                        for axis in ('x', 'y', 'z'):
                            row[key+'_'+axis] = getattr(value, axis)
                        if hasattr(value, 'w'): row[key+'_w'] = value.w
                    else: row[key] = value
            if group not in writers:
                handle = stack.enter_context((folder/(group+'.csv')).open('w', newline=''))
                writers[group] = csv.DictWriter(handle, fieldnames=list(row))
                writers[group].writeheader()
            writers[group].writerow(row)
    print('Wrote '+', '.join(str(folder/(k+'.csv')) for k in writers))


if __name__ == '__main__': main()
