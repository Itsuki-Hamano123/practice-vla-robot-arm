#!/usr/bin/env python3
"""Gazebo 上の全リンクの位置を表示する（姿勢確認用）。"""
import rclpy
from gazebo_msgs.msg import LinkStates

rclpy.init()
n = rclpy.create_node('link_poses')
g = {}
n.create_subscription(LinkStates, '/link_states', lambda m: g.setdefault('m', m), 1)
while 'm' not in g:
    rclpy.spin_once(n, timeout_sec=1)
for nm, p in zip(g['m'].name, g['m'].pose):
    print(f'{nm:35s} {p.position.x:7.3f} {p.position.y:7.3f} {p.position.z:7.3f}')
