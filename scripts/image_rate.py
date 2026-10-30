"""カメラの画像レートと RTF を測る（docs/sim速度設定.md 4章）。sim を起動した状態で実行する。

  docker compose exec -T sim bash -lc 'cd /ws && source /opt/ros/humble/setup.bash && python3 scripts/image_rate.py 20'
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rclpy.init()
n = rclpy.create_node('rate_probe')
data = {t: [] for t in ('/cam_front/image_raw/compressed', '/cam_wrist/image_raw/compressed')}
for t in data:
    n.create_subscription(CompressedImage, t,
                          lambda m, t=t: data[t].append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
                          qos_profile_sensor_data)
end = time.time() + DUR
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
for t, d in data.items():
    if len(d) < 2:
        print(t, 'no data', len(d))
        continue
    wall = d[-1][0] - d[0][0]
    sim = d[-1][1] - d[0][1]
    k = len(d) - 1
    print(f'{t}: n={len(d)} wall={k / wall:.2f}Hz sim={k / sim:.2f}Hz RTF={sim / wall:.3f}')
