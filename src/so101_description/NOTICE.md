# 出典

`urdf/so101.urdf.xacro` の本体部分と `meshes/` は、TheRobotStudio/SO-ARM100 の `Simulation/SO101`
（commit a758567c3978dfeefe282ede0500085a48fe8f78、Apache-2.0）から取り込み、改変したものです。
ライセンス全文は同じフォルダの `LICENSE`。

改変点：メッシュのパスを `package://` に変更、一部リンクの衝突形状を削除、transmission を削除、
カメラ・摩擦・ros2_control・Gazebo プラグインを追加。
