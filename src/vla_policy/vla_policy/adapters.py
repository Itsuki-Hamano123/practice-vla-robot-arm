"""ROS メッセージ ⇔ モデルの入出力の変換（docs/設計.md 6.1 の ObsAdapter / ActionAdapter）。
lerobot には依存しない。関節順は JOINTS（URDF の順）で固定する。
"""
import io
import math

import numpy as np
from PIL import Image

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
# 関節の可動範囲 [rad]（URDF。docs/設計.md 5章）
JOINT_LIMITS = np.array([[-1.92, 1.92], [-1.75, 1.75], [-1.69, 1.69], [-1.66, 1.66], [-2.74, 2.84], [-0.17, 1.75]])
UNIT_SCALE = {'radian': 1.0, 'degree': 180.0 / math.pi}  # rad → 学習データの単位


class ObsAdapter:
    """JointState・CompressedImage → (state, images)。state は学習データの単位、images は {入力キー: HWC uint8 RGB}。"""

    def __init__(self, camera_map, state_unit='degree', image_size=None):
        self.camera_map = camera_map        # {トピック: モデルの入力キー}
        self.scale = UNIT_SCALE[state_unit]
        self.image_size = image_size        # [幅, 高さ]。None ならそのまま（モデル側で 512×512 にパディング付きで縮小される）

    def state(self, joint_msg):
        q = dict(zip(joint_msg.name, joint_msg.position))
        return np.array([q[j] for j in JOINTS], np.float32) * self.scale

    def image(self, msg):
        img = Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
        if self.image_size:
            img = img.resize(tuple(self.image_size), Image.BILINEAR)
        return np.asarray(img)

    def __call__(self, joint_msg, image_msgs):
        return self.state(joint_msg), {key: self.image(image_msgs[topic]) for topic, key in self.camera_map.items()}


class ActionAdapter:
    """モデル出力（学習データの単位）→ 関節目標角 [rad]。可動範囲でクリップし、1ステップの変化量を制限する。"""

    def __init__(self, action_unit='degree', max_step=0.05):
        self.scale = UNIT_SCALE[action_unit]
        self.max_step = max_step            # 1ステップあたりの最大変化 [rad]。None なら制限しない

    def to_rad(self, actions):
        return np.asarray(actions, np.float64) / self.scale

    def __call__(self, target_rad, current_rad):
        cmd = np.clip(target_rad, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        if self.max_step:
            cmd = current_rad + np.clip(cmd - current_rad, -self.max_step, self.max_step)
        return cmd
