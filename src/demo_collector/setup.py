from setuptools import setup

package_name = 'demo_collector'

# lerobot・ikpy は policy コンテナの venv（/opt/venv）にある。ros2 run は venv を使わないので、
# 実行は `python -m demo_collector.collect`（scripts/collect_demo.sh）で行う
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
