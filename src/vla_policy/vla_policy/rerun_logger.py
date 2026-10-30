"""推論に固有の rerun ログ（docs/設計.md 6.3）。トピックから取れるもの（カメラ・関節・指令・status）は
topic_recorder.Recorder が記録するので、ここではモデルに渡した入力・予測・推論時間だけを足す。
"""
import numpy as np
import rerun as rr

from .adapters import JOINTS


class RerunLogger:
    def __init__(self, recorder):
        self.recorder = recorder  # 時刻（sim_time）を共有する

    def log_inference(self, state, images, chunk_rad, control_hz, inference_ms):
        """state・images はモデルに渡した値（学習データの単位・入力キー）。chunk_rad は予測をラジアンにしたもの（クリップ前）。"""
        self.recorder.set_time()
        t0 = self.recorder.last_stamp
        for key, img in images.items():
            rr.log(f'policy/input/{key}', rr.Image(img).compress(jpeg_quality=85))
        for j, v in zip(JOINTS, state):
            rr.log(f'policy/input/state/{j}', rr.Scalars(float(v)))
        rr.log('perf/inference_ms', rr.Scalars(inference_ms))
        # チャンク全体を、実行される予定の時刻に並べる（一時停止中に推論するので、t0 から制御周期ごと）
        times = t0 + np.arange(len(chunk_rad)) / control_hz
        for i, j in enumerate(JOINTS):
            rr.send_columns(f'action/pred/{j}', indexes=[rr.TimeColumn('sim_time', duration=times)],
                            columns=rr.Scalars.columns(scalars=chunk_rad[:, i]))
