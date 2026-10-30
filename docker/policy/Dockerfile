# policy コンテナ：ROS 2 Humble (Python 3.10) + PyTorch + lerobot + rerun
# rclpy は Humble の apt 版が Python 3.10 専用のため、Python は 3.10 のまま使う。
# lerobot は 0.4.x が Python>=3.10 を許容する最後の系列（0.6 系は >=3.12 が必要）なので固定する。
FROM osrf/ros:humble-desktop

ARG LEROBOT_VERSION=0.4.0
# GPU マシンでは --build-arg TORCH_INDEX=https://download.pytorch.org/whl/cu121 を指定する
ARG TORCH_INDEX=https://download.pytorch.org/whl/cpu

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-venv git ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# rclpy を使うため system-site-packages を有効にした venv に入れる
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH=/opt/venv/bin:$PATH

# torch は TORCH_INDEX（CPU/GPU を切り替え）を優先して lerobot と一緒に入れる（別々に入れると後段で PyPI の CUDA 版に置き換わる）
# venv は system-site-packages を見るため、pip が apt 版の古いパッケージ（Pillow 9・scipy 1.8 など）を
# 「インストール済み」とみなして使ってしまう。--ignore-installed で依存はすべて venv 側に入れる（apt 版から使うのは rclpy など ROS だけ）
# - rerun-sdk は lerobot 0.4.0 の要求（>=0.24,<0.27）の中で 0.26.2 に固定（Viewer も同じ版を使う）。0.24 以降は numpy 2 必須。scipy は lerobot の必須依存ではないが transformers が import するので明示する
# - cv_bridge（apt 版）は numpy 1 系向けなので policy では使わない
# - setuptools は colcon（apt 版）の要求 <80 に合わせる
RUN pip install --no-cache-dir --ignore-installed \
        --index-url ${TORCH_INDEX} --extra-index-url https://pypi.org/simple \
        "torch>=2.2.1,<2.8.0" torchvision \
        "lerobot[smolvla]==${LEROBOT_VERSION}" "rerun-sdk==0.26.2" ikpy "scipy>=1.13" "setuptools<80"

WORKDIR /ws
