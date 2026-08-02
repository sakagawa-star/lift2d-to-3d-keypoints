"""LoFTR（kornia 組み込み）の環境疎通スモークテスト。

確認事項（疎通のみ。性能評価は criteria lock 後の実験で行う）:
1. kornia の import と LoFTR 学習済み重み（outdoor）の自動ダウンロード
2. GPU（RTX 5060 Ti / cu128）上でダミー入力の forward が通ること
3. 出力辞書のキーと形状が文書どおりであること（keypoints0/1, confidence, batch_indexes）

実行:
    uv run --project matcher_lab python matcher_lab/loftr_smoke.py
"""
import torch
import kornia
from kornia.feature import LoFTR


def main() -> None:
    print(f"torch {torch.__version__} / kornia {kornia.__version__}")
    print(f"CUDA 利用可: {torch.cuda.is_available()}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type == "cuda":
        print(f"GPU: {torch.cuda.get_device_name(0)}")

    print("LoFTR(pretrained='outdoor') を読み込み中（初回は重みを自動ダウンロード）...")
    matcher = LoFTR(pretrained="outdoor").to(device).eval()
    print("読み込み完了")

    # ダミー入力（グレースケール (N,1,H,W)。実画像での評価はここでは行わない）
    torch.manual_seed(0)
    img0 = torch.rand(1, 1, 480, 640, device=device)
    img1 = torch.rand(1, 1, 480, 640, device=device)
    with torch.inference_mode():
        out = matcher({"image0": img0, "image1": img1})

    print("出力キー:", sorted(out.keys()))
    for k, v in out.items():
        print(f"  {k}: shape={tuple(v.shape)} dtype={v.dtype}")
    print("疎通OK")


if __name__ == "__main__":
    main()
