"""MASt3R の環境疎通スモークテスト。

確認事項（疎通のみ。性能評価は criteria lock 後の実験で行う）:
1. ~/git/mast3r（+ dust3r サブモジュール）が本環境（Python 3.11 / torch cu128）で import できること
2. ローカルチェックポイント（~/data/models/mast3r/）から from_pretrained で読み込めること
3. ダミー画像ペアで inference → fast_reciprocal_NNs が通り、マッチ座標が返ること
   （curope CUDA カーネルは未コンパイル。純 torch フォールバックで動作する想定）

実行:
    uv run --project matcher_lab python matcher_lab/mast3r_smoke.py
"""
import sys
from pathlib import Path

import numpy as np
import torch

MAST3R_ROOT = Path.home() / "git" / "mast3r"
CKPT_PATH = (Path.home() / "data" / "models" / "mast3r"
             / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")
sys.path.insert(0, str(MAST3R_ROOT))
sys.path.insert(0, str(MAST3R_ROOT / "dust3r"))

from mast3r.model import AsymmetricMASt3R  # noqa: E402
from mast3r.fast_nn import fast_reciprocal_NNs  # noqa: E402
from dust3r.inference import inference  # noqa: E402
from dust3r.utils.image import load_images  # noqa: E402


def main() -> None:
    print(f"torch {torch.__version__} / CUDA 利用可: {torch.cuda.is_available()}")
    device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"チェックポイント読み込み中: {CKPT_PATH}")
    model = AsymmetricMASt3R.from_pretrained(str(CKPT_PATH)).to(device).eval()
    print("読み込み完了")

    # ダミー画像ペア（ノイズPNG。実画像での評価はここでは行わない）
    import cv2
    tmp_dir = Path("/tmp/claude-1000/mast3r_smoke")
    tmp_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)
    paths = []
    for i in range(2):
        img = rng.integers(0, 256, size=(384, 512, 3), dtype=np.uint8)
        p = tmp_dir / f"dummy{i}.png"
        cv2.imwrite(str(p), img)
        paths.append(str(p))

    images = load_images(paths, size=512, verbose=False)
    with torch.inference_mode():
        output = inference([tuple(images)], model, device, batch_size=1, verbose=False)

    pred1, pred2 = output["pred1"], output["pred2"]
    desc1 = pred1["desc"].squeeze(0).detach()
    desc2 = pred2["desc"].squeeze(0).detach()
    print(f"desc1: shape={tuple(desc1.shape)} / desc2: shape={tuple(desc2.shape)}")

    matches_im0, matches_im1 = fast_reciprocal_NNs(
        desc1, desc2, subsample_or_initxy1=8, device=device, dist="dot", block_size=2**13)
    print(f"fast_reciprocal_NNs 出力: matches_im0 {matches_im0.shape}, "
          f"matches_im1 {matches_im1.shape}")
    print("疎通OK")


if __name__ == "__main__":
    main()
