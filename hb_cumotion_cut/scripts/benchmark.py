#!/usr/bin/env python3
"""Time plan-only CutProfile goals against the single currently running server."""
import argparse
import json
import time

import rclpy
from hb_robot_interfaces.action import CutProfile
from rclpy.action import ActionClient
from rosidl_runtime_py.set_message import set_message_fields


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('goal', help='JSON file with the same goal for both backends')
    parser.add_argument('--backend', choices=['cpu', 'gpu'], required=True)
    parser.add_argument('--runs', type=int, default=5)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('runs must be positive')
    with open(args.goal, encoding='utf-8') as stream:
        fields = json.load(stream)
    goal = CutProfile.Goal()
    set_message_fields(goal, fields)
    if goal.execute or goal.plan_id:
        parser.error('Benchmark requires a plan-only goal with an empty plan_id')
    rclpy.init()
    node = rclpy.create_node('cut_profile_benchmark')
    client = ActionClient(node, CutProfile, '/cut_profile')
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('/cut_profile is unavailable')
        for run in range(args.runs):
            started = time.perf_counter()
            sent = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, sent)
            handle = sent.result()
            record = dict(backend=args.backend, run=run + 1, goal=fields, accepted=handle.accepted)
            if handle.accepted:
                done = handle.get_result_async()
                rclpy.spin_until_future_complete(node, done)
                reply = done.result()
                record.update(status=reply.status, success=reply.result.success,
                              plan_id=reply.result.plan_id, message=reply.result.message,
                              segments=list(reply.result.required_segments))
            record['wall_s'] = time.perf_counter() - started
            print(json.dumps(record), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
