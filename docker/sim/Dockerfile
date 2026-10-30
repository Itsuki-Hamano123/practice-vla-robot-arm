# sim コンテナ：講義イメージ（ROS 2 Humble + Gazebo）をベースにする
# Gazebo の版は `gazebo --version` / `gz sim --version` で確認し、docs/設計.md に反映する
ARG BASE_IMAGE=airobotbook/ros2-desktop-ai-robot-book-humble
FROM ${BASE_IMAGE}

# 講義イメージの ROS apt 鍵は期限切れ（2025 年に更新済み）なので、最新の鍵に差し替える
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

# 追加が必要になりそうなパッケージ（実際に必要なものだけ残す）
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gazebo-ros2-control \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
