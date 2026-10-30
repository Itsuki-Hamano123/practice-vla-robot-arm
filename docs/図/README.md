# 構成図（draw.io）

内容は [設計.md](../設計.md) を図にしたもの。設計を変えたら、`.drawio` を直して `.svg` を書き出し直す。

| 図 | 内容 |
| --- | --- |
| [00_課題の全体像](00_課題の全体像.svg) | 課題のゴール・進め方・rerun での可視化 |
| [01_システム構成](01_システム構成.svg) | sim / policy コンテナの中身と ROS トピックのやり取り |
| [02_ライブラリ依存](02_ライブラリ依存.svg) | ROS パッケージと使用ライブラリ・実行基盤の関係 |
| [03_デモ収集フロー](03_デモ収集フロー.svg) | デモ自動収集 → デモデータセット保存 → 目視確認 |
| [04_ファインチューニングフロー](04_ファインチューニングフロー.svg) | `scripts/train.sh` → `lerobot-train` → チェックポイント |
| [05_推論実行フロー](05_推論実行フロー.svg) | `vla_policy_node` の制御ループ（観測 → SmolVLA → 指令） |
| [06_評価フロー](06_評価フロー.svg) | `scripts/eval_ckpt.sh` によるチェックポイントごとの評価 → 結果の記録・採用 |
| [07_技術スタックの流れ](07_技術スタックの流れ.svg) | 使った技術（ROS 2・Gazebo・LeRobot・SmolVLA・rerun・ikpy）が推論と Phase B のどこで働くか |

色：青＝sim、緑＝policy、橙＝GPU・モデル本体、黄＝データ・ファイル、紫＝人・スクリプト、灰＝外部ライブラリ。

## 編集と書き出し

- 編集：`.drawio` を VS Code の draw.io 拡張か [diagrams.net](https://app.diagrams.net/) で開く
- 書き出し（`.svg` はすべてここから作る。ラベルは文字のまま入る）：

```bash
docker run --rm -e HOME=/tmp -v "$PWD/docs:/data" rlespinasse/drawio-export \
  -f svg --remove-page-suffix -o svg 図
# docs/図/svg/ に出る（root 所有）。中身を docs/図/ へコピーして svg/ を消す
```
