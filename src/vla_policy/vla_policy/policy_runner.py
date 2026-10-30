"""SmolVLA の読み込みと推論。**lerobot の API に依存するコードはこのファイルに閉じ込める**。
lerobot 0.4.0 の前処理・後処理パイプライン（policy_preprocessor.json / policy_postprocessor.json）を使う。
"""
import numpy as np
import torch
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

NORMALIZER = 'normalizer_processor'
UNNORMALIZER = 'unnormalizer_processor'


class PolicyRunner:
    def __init__(self, model_path, device='cpu', norm_stats=None, num_threads=None):
        """norm_stats：正規化の統計を、保存されている統計のうちこの名前のもので置き換える。
        smolvla_base の統計はデータセット名付きのキー（'so100.buffer.action' など）で保存されていて、
        そのままだと 'action' に当たらず正規化されない。state の統計は保存されていないので action のもので代用する
        （SO-100 系は state と action が同じ関節空間・単位）。None なら保存されている統計をそのまま使う（fine-tune 済みモデル）。"""
        if num_threads:
            torch.set_num_threads(num_threads)
        self.device = device
        self.policy = SmolVLAPolicy.from_pretrained(model_path)
        self.policy.config.device = device
        self.policy.to(device).eval()
        dev = {'device_processor': {'device': device}}
        pre_over, post_over = dict(dev), {}
        if norm_stats:
            pre, _ = make_pre_post_processors(self.policy.config, model_path, preprocessor_overrides=dev)
            saved = next(s for s in pre.steps if type(s).__name__ == 'NormalizerProcessorStep').stats
            key = f'{norm_stats}.buffer.action'
            if key not in saved:
                raise KeyError(f'統計 {key} が無い（あるのは {list(saved)}）')
            act = {k: np.asarray(v) for k, v in saved[key].items()}
            pre_over[NORMALIZER] = {'stats': {'action': act, 'observation.state': act}}
            post_over[UNNORMALIZER] = {'stats': {'action': act}}
        self.pre, self.post = make_pre_post_processors(
            self.policy.config, model_path, preprocessor_overrides=pre_over, postprocessor_overrides=post_over)
        self.image_keys = list(self.policy.config.image_features)
        self.chunk_size = self.policy.config.chunk_size

    def infer(self, state, images, task):
        """state：(6,) 学習データの単位、images：{入力キー: HWC uint8}、task：指示文。
        戻り値：行動チャンク (chunk_size, 6)（学習データの単位）。"""
        obs = {'observation.state': torch.from_numpy(np.asarray(state, np.float32)), 'task': task}
        for key, img in images.items():
            obs[key] = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1).float() / 255.0
        batch = self.pre(obs)
        with torch.no_grad():
            chunk = self.policy.predict_action_chunk(batch)
        return self.post(chunk)[0].cpu().numpy()
