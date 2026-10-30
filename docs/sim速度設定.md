# sim の速度設定（fps・RTF・カメラ描画）

sim の速さを CPU 環境と GPU 環境で変えている。その値と理由、測り方をまとめる。決定事項は [設計.md](設計.md) の決定事項ログ（2026-10-24）。

## 0. 設定値の一覧（CPU / GPU）

| 項目 | CPU（ノートPC。GPU なし） | GPU（WSL2 + NVIDIA GPU） | 場所 |
| --- | --- | --- | --- |
| 起動 | `docker compose up -d --build` | `docker compose -f compose.yaml -f compose.gpu.yaml up -d --build` | [compose.gpu.yaml](../compose.gpu.yaml) |
| sim の描画器 | llvmpipe（CPU のソフトウェア描画） | `D3D12 (NVIDIA GPU)`（Mesa の d3d12。`/dev/dxg` と `/usr/lib/wsl` を渡す） | compose.gpu.yaml の `sim` |
| `real_time_update_rate` | **80**（RTF 約 0.16） | **500**（RTF 1.0） | CPU：[table.world](../src/so101_gazebo/worlds/table.world)。GPU：環境変数 `SIM_REAL_TIME_UPDATE_RATE`（compose.gpu.yaml。[sim.launch.py](../src/so101_gazebo/launch/sim.launch.py) が world を書き換えて起動する） |
| カメラの描画レート（壁時計） | 約 1.5〜2 枚/秒 | 30 枚/秒（`update_rate` 30 の上限） | 実測（4 章） |
| 画像レート（sim 時間） | 約 9.7Hz | 30Hz | 実測 |
| カメラの `update_rate` | 30 | 30 | [so101.urdf.xacro](../src/so101_description/urdf/so101.urdf.xacro) |
| データセットの fps | 10（sim 時間） | 10（変えない。2 章） | 設計.md 6.2・8.1 |
| policy の torch | 2.7.1+cpu（`TORCH_INDEX` 既定） | 2.7.1+cu126（`TORCH_INDEX=…/whl/cu126`） | [docker/policy/Dockerfile](../docker/policy/Dockerfile)・compose.gpu.yaml |
| 推論の設定ファイル | [policy_base.yaml](../config/policy_base.yaml)（`device: cpu`） | [policy_base_gpu.yaml](../config/policy_base_gpu.yaml)（`device: cuda`。他は同じ） | `scripts/run_policy.sh <設定ファイル>` |
| 推論時間（smolvla_base、1 チャンク） | 約 25〜35 秒 | 約 0.25〜0.5 秒（初回は約 0.9 秒） | 実測（vla_policy のログ） |
| デモ収集 1 エピソードの壁時計 | 約 80〜90 秒 | 約 11〜12 秒 | 実測（`collect_demo.sh` の `wall_sec`） |

CPU と GPU のどちらでも、データセットの中身（sim 時間の 10Hz のフレーム）は同じになる。違うのは壁時計の速さだけ。

## 1. CPU の設定と理由

| 項目 | CPU の値 | 場所 |
| --- | --- | --- |
| Phase B のデータセット fps・制御周期 | 10Hz（sim 時間） | 設計.md 6.2・8.1 |
| `real_time_update_rate` | **80**（物理ステップ 0.002 秒 → RTF 約 0.16） | [table.world](../src/so101_gazebo/worlds/table.world)（元は 500 で RTF 約 0.87〜1） |
| カメラの `update_rate` | 30（URDF） | [so101.urdf.xacro](../src/so101_description/urdf/so101.urdf.xacro) |

**sim を 0.16 倍に遅くしているのは、CPU（ソフトウェア描画）でカメラ画像を作る処理が遅いため。** 物理計算が遅いわけではない。

- sim のコンテナには `/dev/dri` が無く、GPU を使えない。カメラの描画は CPU のソフトウェア描画になる（gzserver が約 2.4 コア相当を使う）。
- カメラの描画は **壁時計で約 2 枚/秒**。解像度を 640×480 から 256×192 に下げても変わらなかった（画素数に依存しない固定コストが大きい。原因の特定は未了。影・カメラ台数・描画器は未確認）。
- 画像が出る間隔は壁時計で決まるので、**sim 時間あたりの画像枚数は RTF に反比例する**。

  ```
  sim 時間での画像レート [Hz] ＝ 描画レート [枚/秒（壁時計）] ÷ RTF
  必要な RTF ＝ 描画レート ÷ fps
  ```

- 実測（正面カメラ。測り方は 4 章）：

  | RTF | 壁時計での画像レート | sim 時間での画像レート |
  | --- | --- | --- |
  | 0.87（`real_time_update_rate` 500） | 約 1.9Hz | 約 2.1Hz |
  | 0.20（同 100） | 約 1.7Hz | 約 8.5Hz |
  | 約 0.16（同 80、**CPU の設定**） | 約 1.5Hz | 約 9.7Hz |

- fps と RTF の関係（描画レート 2 枚/秒のとき）：

  | fps | 必要な RTF | 100 件（1 エピソード sim で 10 秒）の壁時計 |
  | --- | --- | --- |
  | 30 | 約 0.06 | 約 4 時間 |
  | **10（採用）** | 約 0.16〜0.2 | 約 1.7 時間 |

データに残るのは sim 時間のタイムスタンプだけなので、sim を遅くしても学習データの中身は変わらない。

## 2. GPU の設定と理由

GPU で描画すると、カメラ 2 台とも壁時計で 30 枚/秒（`update_rate` の上限）出る。そのため sim を遅くする必要がなく、RTF 1（`real_time_update_rate` 500）で動かす。

- 実測（2026-10-24、WSL2 + NVIDIA GPU。正面・手首とも同じ値）：

  | `real_time_update_rate` | RTF | 壁時計での画像レート | sim 時間での画像レート |
  | --- | --- | --- | --- |
  | 80 | 0.16 | 5.8Hz | 36Hz（`update_rate` 30 を超えて出た。原因は未確認） |
  | 250 | 0.50 | 15Hz | 30Hz |
  | **500（GPU の設定）** | 1.00 | 30Hz | 30Hz |

- WSL2 では NVIDIA の GLX ドライバが使えない。OpenGL は Mesa の d3d12 ドライバから `/dev/dxg` を通して GPU を使う。そのため compose.gpu.yaml で sim に `/dev/dxg`・`/dev/dri`・`/usr/lib/wsl` を渡し、`LD_LIBRARY_PATH=/usr/lib/wsl/lib`・`GALLIUM_DRIVER=d3d12` を設定している。講義イメージの Mesa 23.2 には d3d12 ドライバが入っているので、イメージへの追加は要らない。描画器は `glxinfo -B` の `OpenGL renderer string` で確認する
- policy の torch は CUDA 版にする。cu121 の index には torch 2.5 系までしか無く、CPU 版と同じ 2.7.1 を入れられないので cu126 を使う
- RTF は 1 が上限（物理ステップ 0.002 秒を実時間で回す）。1 を超える設定は物理の都合で現実的でない

**fps は変えない**。fps は学習データを収集した時点で決まる値で、推論も同じ周期（sim 時間）で動かす必要がある。fps を 30 に戻す場合は、データを撮り直す（または新しいデータセットとして撮る）ことになる。GPU では画像が 30Hz 出るので、30Hz のデータセットも撮れる。

学習だけを GPU マシンで行う場合、sim は使わないので、この設定は何も変えなくてよい。

### 2.1 GPU で確認したこと（2026-10-24）

- smolvla_base（`policy_base_gpu.yaml`）で、赤・青の指示を 1 エピソードずつ動かした。アームは動くが箱は掴めない（`lifted` が false のまま `timeout`）。ファインチューニング 前の挙動。推論時間は平均 433〜436ms
- デモ収集（`collect_demo.sh --episodes 2`）は 2 件とも成功し、重複フレームは 0 だった。1 エピソードで届く画像は約 300 枚（30Hz × sim 約 10 秒）。収集スクリプトの上限は、もとは画像の枚数（`max_frames=400`。CPU では sim 約 40 秒、GPU では約 13.3 秒に相当）だった。画像レートで意味が変わるので、sim 時間の秒数（`max_sec=20`。CPU・GPU 共通）に置き換えた。軌道は最長でも約 11.9 秒なので、通常は軌道の終わりで止まる

## 3. 設定を変えたときに合わせて直すもの

- 推論の制御周期：[policy_base.yaml](../config/policy_base.yaml) の `control_hz`（ファインチューニング 後のモデルでは 10）。
- `max_step`（1 ステップあたりの関節の最大変化 [rad]）：制御周期に比例させる。30Hz で 0.05 → 10Hz で 0.15。
- Phase A（smolvla_base）の推論は 30Hz 制御のまま。画像が 2Hz 程度しか更新されない状態で動いたので、古い画像で動く時間が長かった。
- **推論ノード（vla_policy）の制御周期は壁時計で刻んでいる**（`time.monotonic`）。sim 時間での周期は `control_hz ÷ RTF` になる。GPU（RTF 1）では設定どおりだが、CPU（RTF 0.16）では sim 時間で約 6 倍速く指令を送ることになる（`control_hz: 10` なら sim 時間で約 62Hz）。ファインチューニング 後のモデルを CPU で評価する場合は、その前に sim 時間で刻むように直す必要がある。評価は GPU マシンで行うので、今は直さない

## 4. 画像レートの測り方

sim コンテナ内で、`/cam_front/image_raw/compressed` と `/clock` の stamp と壁時計を 20 秒間記録し、次を計算する（`/clock` は QoS が合わず受信できないことがあるので、画像の stamp から求める）。

```
壁時計での画像レート ＝ 受信枚数 ÷ 壁時計の経過時間
sim 時間での画像レート ＝ 受信枚数 ÷ stamp の経過時間
RTF ＝ stamp の経過時間 ÷ 壁時計の経過時間
```

`scripts/image_rate.py` がこの計算をする（`docker compose exec -T sim bash -lc 'cd /ws && source /opt/ros/humble/setup.bash && python3 scripts/image_rate.py 20'`。引数は測る秒数）。

sim の起動からトピックが出るまで 1 分前後かかる（GPU では数秒）。`scripts/sim_restart.sh` で再起動したあと、`ros2 topic list` に画像トピックが出てから測る。

## 5. 未確認のこと

- CPU で描画が遅い原因（影・カメラ 2 台の直列描画・Gazebo 内部の待ち合わせなど）。影を切る、カメラを 1 台にして測れば切り分けられる。描画レートが上がれば CPU のままでも RTF を上げられる。
- RTF を落としたときの把持（疑似固定）への影響。ステップ幅は変えていないので、影響はない見込み。GPU（RTF 1）ではデモ収集の把持が 2 件とも成功した。
- GPU で `sync_mode: false`（推論中も sim を止めない）にしたときの挙動。推論は約 0.3 秒なので、1 チャンク（30Hz で 50 ステップ ≒ 1.7 秒）より短い。`policy_base_gpu.yaml` は false にした（チャンクを全部実行してから次を推論する逐次の流れ）。実行して挙動を確認したら、ここに結果を書く。
