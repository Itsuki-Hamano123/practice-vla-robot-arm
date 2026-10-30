from setuptools import setup

package_name = 'vla_policy'

# torch・lerobot・rerun は policy コンテナの venv（/opt/venv）にある。ros2 run は /usr/bin/python3 で起動して
# venv を使わないので、実行は `python -m vla_policy.vla_policy_node`（scripts/run_policy.sh）で行う
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
