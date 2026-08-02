"""feat-026 M3b: 学習ベースマッチャー CLI(matcher_lab 環境で実行)

criteria.md(locked 2026-08-02) §3 のマッチャー仕様を実装する。phase4 環境のドライバ
(m3b_run.py)から subprocess で呼ばれ、画像ペア1組のマッチ座標(原寸 1920x1080)を
NPZ で返す。判定は行わない(ドライバ側の責務)。

実行方法(プロジェクトルートで):
    uv run --project matcher_lab python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/matcher_cli.py \
        query.png render.png --matcher loftr_outdoor --out matches.npz [--resolution full]

--matcher: loftr_outdoor / loftr_indoor_new / mast3r
--resolution: full(既定。LoFTR のみ有効) / 1280x720(OOM fallback。criteria §3.1)
出力 NPZ: u_q, u_r(原寸座標 float64 (N,2)), conf(float32 (N,)。MASt3R は 1.0 埋め),
          meta(JSON 文字列: 解像度・逆写像係数・round-trip 誤差・原寸範囲内率)
終了コード: 0=成功 / 42=CUDA OOM(ドライバはこれを検知して fallback に切替える) / 1=その他
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

MAST3R_ROOT = Path.home() / "git" / "mast3r"
MAST3R_CKPT = (Path.home() / "data" / "models" / "mast3r"
               / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")

ORIG_W, ORIG_H = 1920, 1080  # criteria §2(入力は原寸 1920x1080 で固定)
FALLBACK_W, FALLBACK_H = 1280, 720  # criteria §3.1
GRID_N = 5  # round-trip 検証グリッド 5x5=25点(criteria フェーズ0(c))


def roundtrip_and_range(fwd, inv, u_orig_list) -> dict:
    """逆写像の検証値を計算(criteria フェーズ0(c) の数値基準の材料)。

    fwd: 原寸→縮小座標, inv: 縮小→原寸座標。
    u_orig_list: 逆写像済みマッチ座標(原寸)のリスト(範囲内率の計算対象)。
    """
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


def run_loftr(img_q_bgr, img_r_bgr, weight: str, resolution: str, device):
    """LoFTR(kornia)。戻り値: (u_q, u_r, conf, meta)。座標は原寸"""
    from kornia.feature import LoFTR
    gray_q = cv2.cvtColor(img_q_bgr, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r_bgr, cv2.COLOR_BGR2GRAY)
    if resolution == "full":
        scale = 1.0
        proc_q, proc_r = gray_q, gray_r
    else:  # 1280x720(criteria §3.1: 係数 1.5 で原寸へ逆写像)
        scale = FALLBACK_W / ORIG_W  # = 1/1.5
        proc_q = cv2.resize(gray_q, (FALLBACK_W, FALLBACK_H), interpolation=cv2.INTER_AREA)
        proc_r = cv2.resize(gray_r, (FALLBACK_W, FALLBACK_H), interpolation=cv2.INTER_AREA)

    def to_tensor(g):
        return torch.from_numpy(g).float().div(255.0)[None, None].to(device)

    matcher = LoFTR(pretrained=weight).to(device).eval()
    with torch.inference_mode():
        out = matcher({"image0": to_tensor(proc_q), "image1": to_tensor(proc_r)})
    kq = out["keypoints0"].cpu().numpy().astype(np.float64)
    kr = out["keypoints1"].cpu().numpy().astype(np.float64)
    conf = out["confidence"].cpu().numpy().astype(np.float32)

    # 逆写像(画素中心規約): orig = (small + 0.5)/scale - 0.5
    def fwd(u):
        return (u + 0.5) * scale - 0.5

    def inv(u):
        return (u + 0.5) / scale - 0.5

    u_q, u_r = inv(kq), inv(kr)
    meta = {"matcher": f"loftr_{weight}", "resolution": resolution, "scale": scale}
    meta.update(roundtrip_and_range(fwd, inv, np.vstack([u_q, u_r]) if len(u_q) else []))
    return u_q, u_r, conf, meta


def run_mast3r(query_path: str, render_path: str, device):
    """MASt3R。戻り値: (u_q, u_r, conf=1埋め, meta)。座標は原寸。

    criteria §3.2: load_images(size=512) + fast_reciprocal_NNs(README 既定値)。
    逆写像式は実際の true_shape・リサイズ規則から導出して meta に記録する。
    """
    sys.path.insert(0, str(MAST3R_ROOT))
    sys.path.insert(0, str(MAST3R_ROOT / "dust3r"))
    from mast3r.model import AsymmetricMASt3R
    from mast3r.fast_nn import fast_reciprocal_NNs
    from dust3r.inference import inference
    from dust3r.utils.image import load_images

    model = AsymmetricMASt3R.from_pretrained(str(MAST3R_CKPT)).to(device).eval()
    images = load_images([query_path, render_path], size=512, verbose=False)
    with torch.inference_mode():
        output = inference([tuple(images)], model, device, batch_size=1, verbose=False)
    desc1 = output["pred1"]["desc"].squeeze(0).detach()
    desc2 = output["pred2"]["desc"].squeeze(0).detach()
    m0, m1 = fast_reciprocal_NNs(desc1, desc2, subsample_or_initxy1=8,
                                 device=device, dist="dot", block_size=2**13)

    # 逆写像式の導出: load_images は長辺512へリサイズ(round)後、中心で16の倍数へ切り詰め
    h2, w2 = (int(v) for v in images[0]["true_shape"].flatten())
    s = 512 / max(ORIG_W, ORIG_H)
    wr, hr = round(ORIG_W * s), round(ORIG_H * s)  # リサイズ直後(切り詰め前)
    ox, oy = (wr - w2) // 2, (hr - h2) // 2        # 中心切り詰めのオフセット
    sx, sy = wr / ORIG_W, hr / ORIG_H

    def fwd(u):
        return np.stack([(u[:, 0] + 0.5) * sx - 0.5 - ox,
                         (u[:, 1] + 0.5) * sy - 0.5 - oy], axis=1)

    def inv(u):
        return np.stack([(u[:, 0] + ox + 0.5) / sx - 0.5,
                         (u[:, 1] + oy + 0.5) / sy - 0.5], axis=1)

    u_q = inv(np.asarray(m0, dtype=np.float64).reshape(-1, 2))
    u_r = inv(np.asarray(m1, dtype=np.float64).reshape(-1, 2))
    conf = np.ones(len(u_q), dtype=np.float32)
    meta = {"matcher": "mast3r", "resolution": f"{w2}x{h2}",
            "true_shape": [h2, w2], "resized_precrop": [hr, wr],
            "offsets": [ox, oy], "scale_xy": [sx, sy]}
    meta.update(roundtrip_and_range(fwd, inv, np.vstack([u_q, u_r]) if len(u_q) else []))
    return u_q, u_r, conf, meta


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("query")
    ap.add_argument("render")
    ap.add_argument("--matcher", required=True,
                    choices=["loftr_outdoor", "loftr_indoor_new", "mast3r"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--resolution", default="full", choices=["full", "1280x720"])
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    img_q = cv2.imread(args.query, cv2.IMREAD_COLOR)
    img_r = cv2.imread(args.render, cv2.IMREAD_COLOR)
    if img_q is None or img_r is None:
        print(f"画像が読めない: {args.query} / {args.render}", file=sys.stderr)
        return 1
    for name, im in (("query", img_q), ("render", img_r)):
        if im.shape[:2] != (ORIG_H, ORIG_W):
            print(f"{name} のサイズ {im.shape[:2]} が原寸 ({ORIG_H},{ORIG_W}) と不一致",
                  file=sys.stderr)
            return 1

    try:
        if args.matcher == "mast3r":
            u_q, u_r, conf, meta = run_mast3r(args.query, args.render, device)
        else:
            weight = args.matcher.removeprefix("loftr_")
            u_q, u_r, conf, meta = run_loftr(img_q, img_r, weight, args.resolution, device)
    except torch.cuda.OutOfMemoryError as e:
        print(f"CUDA OOM: {e}", file=sys.stderr)
        return 42

    np.savez(args.out, u_q=u_q, u_r=u_r, conf=conf, meta=json.dumps(meta))
    print(f"{meta['matcher']}: {len(u_q)} マッチ / meta={json.dumps(meta)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
