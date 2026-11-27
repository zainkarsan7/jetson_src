#!/usr/bin/env python3
"""Plot feature-normal depth, diagnostic ray categories, and authoritative bins.
Run live or while playing a bag; never sends motion/service requests.
"""
from collections import deque
import argparse
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from hb_robot_interfaces.msg import KerfDepthProfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--feature', default='')
    parser.add_argument('--frames', type=int, default=20)
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.frames <= 100: parser.error('--frames must be 1..100')
    rclpy.init(args=ros_args); node = Node('kerf_depth_plot')
    data = deque(maxlen=args.frames); identity = [None]; dirty = [False]

    def receive(m):
        if args.feature and m.feature_id != args.feature: return
        key = (m.cut_id, m.feature_id, m.geometry_version, m.pose_version, m.execution_version)
        if key != identity[0]: data.clear(); identity[0] = key
        data.append(m); dirty[0] = True

    node.create_subscription(KerfDepthProfile, '/inspect_cut_server/depth_profile', receive, qos_profile_sensor_data)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6), gridspec_kw={'height_ratios': [4, 1]})
    plt.ion(); plt.show()
    colors = ['gray', 'seagreen', 'firebrick']; names = ['UNKNOWN', 'OPEN', 'MATERIAL']
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.05)
            if data and dirty[0]:
                dirty[0] = False
                for ax in axes: ax.clear()
                s = np.concatenate([m.s for m in data]); d = 1000*np.concatenate([m.measured_depth_m for m in data])
                kinds = np.concatenate([m.classification for m in data])
                strict = np.concatenate([m.within_kerf for m in data]).astype(bool)
                for kind in range(3):
                    mask = (kinds == kind) & np.isfinite(d)
                    axes[0].scatter(s[mask], d[mask], s=2, alpha=0.35, c=colors[kind], label=names[kind])
                m = data[-1]; order = np.argsort(m.s)
                for values, label in [(m.material_entry_m, 'Model front'), (m.material_exit_m, 'Model back')]:
                    axes[0].plot(np.asarray(m.s)[order], 1000*np.asarray(values)[order], '.', markersize=1, label=label)
                axes[0].set_ylabel('Inward feature-normal depth [mm]'); axes[0].legend(loc='best')
                axes[0].set_title(f'{m.feature_id} | {len(data)} frames | {np.count_nonzero(~np.isfinite(d))} no-return rays (UNKNOWN)\n'
                                  f'{np.count_nonzero(strict)} strict-slit rays; broad-band points are diagnostic')
                for kind in range(3):
                    mask = np.asarray(m.bin_classification) == kind
                    axes[1].scatter(np.asarray(m.bin_s)[mask], np.full(mask.sum(), kind), c=colors[kind], marker='s')
                axes[1].set_yticks([0, 1, 2], names); axes[1].set_ylim(-0.5, 2.5)
                axes[1].set_ylabel('Fused bins'); axes[1].set_xlabel('Selected cut feature s (trimmed start=0, end=1)')
                axes[1].set_xlim(0, 1); fig.tight_layout(); fig.canvas.draw_idle()
            plt.pause(0.05)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
