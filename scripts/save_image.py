#!/usr/bin/env python3
"""画像トピックを1枚だけ PNG に保存する（カメラ位置の確認用）。
使い方: python3 scripts/save_image.py /cam_front/image_raw outputs/cam_front.png
"""
import sys

import numpy as np
import rclpy
from PIL import Image as PILImage
from sensor_msgs.msg import Image


def main():
    topic, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('save_image')
    got = {}
    node.create_subscription(Image, topic, lambda m: got.setdefault('m', m), 1)
    while 'm' not in got and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    m = got['m']
    arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    PILImage.fromarray(arr).save(out)
    print(f'saved {out} {m.width}x{m.height} {m.encoding}')


if __name__ == '__main__':
    main()
