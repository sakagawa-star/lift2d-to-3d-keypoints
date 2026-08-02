"""feat-026: LoFTR推論CLI（matcher_lab環境で実行）

phase4環境のドライバ（phase4/refine_extrinsics.py）からsubprocessで呼ばれ、画像ペア
1組のマッチ座標（原寸1920x1080）をNPZで返す。判定・ゲート適用はドライバ側の責務で、
本CLIはマッチ座標とconfidence、変換メタ情報を返すのみ。

design.md §4.9/§6.3 に基づく正式版。M3b の matcher_cli.py（LoFTR部分）を実測根拠として
参照するが、本ファイルはゼロから書き直す（MASt3R部分は移植しない）。

実行方法（プロジェクトルートで）:
    uv run --project matcher_lab python matcher_lab/loftr_cli.py \\
        query.png render.png --out matches.npz

終了コード: 0=成功 / 42=CUDA OOM / 1=その他（画像不正・重み欠落）
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

ORIG_W, ORIG_H = 1920, 1080          # design §5 制約: 対応画像サイズは原寸のみ
FALLBACK_W, FALLBACK_H = 1280, 720   # design §3.1: LoFTR入力解像度（固定。fullは使わない）
GRID_N = 5                           # round-trip検証グリッド 5x5=25点

# design §3.1: LoFTR重み（ローカルキャッシュ固定。sha256はM3b criteria実測値と同一）
WEIGHT_PATH = Path.home() / ".cache" / "torch" / "hub" / "checkpoints" / "loftr_outdoor.ckpt"
WEIGHT_SHA256 = "21f5bec5968178e8bc8b7633441836fe5de4f47d861dd2cd7dc38e271b0479ec"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_weight_offline() -> None:
    """重みのローカルキャッシュ存在・sha256一致を検証する（NFR-005: 自動DLへのfallbackなし）"""
    if not WEIGHT_PATH.exists():
        print(
            f"エラー: LoFTR outdoor重みがローカルキャッシュにありません: {WEIGHT_PATH}\n"
            "README記載のセットアップ手順（重みダウンロードコマンド）を実行してください。",
            file=sys.stderr,
        )
        raise SystemExit(1)
    got = sha256_file(WEIGHT_PATH)
    if got != WEIGHT_SHA256:
        print(
            f"エラー: LoFTR outdoor重みのsha256が一致しません: {WEIGHT_PATH}\n"
            f"  期待: {WEIGHT_SHA256}\n  実際: {got}\n"
            "README記載のセットアップ手順（重みダウンロードコマンド）を実行してください。",
            file=sys.stderr,
        )
        raise SystemExit(1)


def roundtrip_and_range(fwd, inv, u_orig_list) -> dict:
    """逆写像の検証値（round-trip最大誤差・原寸範囲内率）を計算する（design §3.1「逆写像検証」行）"""
    gx = np.linspace(0, ORIG_W - 1, GRID_N)
    gy = np.linspace(0, ORIG_H - 1, GRID_N)
    grid = np.array([[x, y] for x in gx for y in gy], dtype=np.float64)
    rt_err = float(np.abs(inv(fwd(grid)) - grid).max())
    if len(u_orig_list) > 0:
        u = np.asarray(u_orig_list, dtype=np.float64)
        in_range = float(np.mean((u[:, 0] >= 0) & (u[:, 0] <= ORIG_W - 1)
                                 & (u[:, 1] >= 0) & (u[:, 1] <= ORIG_H - 1)))
    else:
        in_range = 1.0
    return {"roundtrip_max_px": rt_err, "in_range_rate": in_range}


def run_loftr(img_q_bgr: np.ndarray, img_r_bgr: np.ndarray, device):
    """LoFTR（kornia、outdoor重み）推論。戻り値: (u_q, u_r, conf, meta)。座標は原寸"""
    from kornia.feature import LoFTR

    gray_q = cv2.cvtColor(img_q_bgr, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r_bgr, cv2.COLOR_BGR2GRAY)
    scale = FALLBACK_W / ORIG_W  # design §3.1: 係数1.5で原寸へ逆写像(= 1/1.5 = 2/3)
    proc_q = cv2.resize(gray_q, (FALLBACK_W, FALLBACK_H), interpolation=cv2.INTER_AREA)
    proc_r = cv2.resize(gray_r, (FALLBACK_W, FALLBACK_H), interpolation=cv2.INTER_AREA)

    def to_tensor(g):
        return torch.from_numpy(g).float().div(255.0)[None, None].to(device)

    matcher = LoFTR(pretrained="outdoor").to(device).eval()
    with torch.inference_mode():
        out = matcher({"image0": to_tensor(proc_q), "image1": to_tensor(proc_r)})
    kq = out["keypoints0"].cpu().numpy().astype(np.float64)
    kr = out["keypoints1"].cpu().numpy().astype(np.float64)
    conf = out["confidence"].cpu().numpy().astype(np.float32)

    # 逆写像（画素中心規約）: orig = (small + 0.5)/scale - 0.5（design §3.1「座標逆写像」行）
    def fwd(u):
        return (u + 0.5) * scale - 0.5

    def inv(u):
        return (u + 0.5) / scale - 0.5

    u_q, u_r = inv(kq), inv(kr)
    meta = {"resolution": f"{FALLBACK_W}x{FALLBACK_H}", "scale": scale,
            "weight_sha256": WEIGHT_SHA256}
    meta.update(roundtrip_and_range(fwd, inv, np.vstack([u_q, u_r]) if len(u_q) else []))
    return u_q, u_r, conf, meta


def main() -> int:
    ap = argparse.ArgumentParser(description="LoFTR(kornia, outdoor重み)推論CLI（feat-026）")
    ap.add_argument("query", help="実写画像PNG（原寸1920x1080）")
    ap.add_argument("render", help="レンダ画像PNG（原寸1920x1080）")
    ap.add_argument("--out", required=True, help="出力NPZパス")
    args = ap.parse_args()

    img_q = cv2.imread(args.query, cv2.IMREAD_COLOR)
    img_r = cv2.imread(args.render, cv2.IMREAD_COLOR)
    if img_q is None or img_r is None:
        print(f"画像が読めません: {args.query} / {args.render}", file=sys.stderr)
        return 1
    for name, im in (("query", img_q), ("render", img_r)):
        if im.shape[:2] != (ORIG_H, ORIG_W):
            print(f"{name} のサイズ {im.shape[:2]} が原寸 ({ORIG_H},{ORIG_W}) と不一致です",
                  file=sys.stderr)
            return 1

    try:
        check_weight_offline()
    except SystemExit as e:
        return int(e.code)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    try:
        u_q, u_r, conf, meta = run_loftr(img_q, img_r, device)
    except torch.cuda.OutOfMemoryError as e:
        print(f"CUDA OOM: {e}", file=sys.stderr)
        return 42

    np.savez(args.out, u_q=u_q, u_r=u_r, conf=conf, meta=json.dumps(meta))
    print(f"loftr_outdoor: {len(u_q)} マッチ / meta={json.dumps(meta)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
