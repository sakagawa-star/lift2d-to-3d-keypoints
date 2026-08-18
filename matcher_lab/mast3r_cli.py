"""feat-035: MASt3R ペアマッチング CLI（matcher_lab 環境で実行）

design.md §2.1 に基づく正式版。phase4 の adjust_extrinsics_multiview.py（Stage P）から
subprocess で呼ばれ、画像ペア1組のマッチ座標（原寸1920x1080）を NPZ で返す。判定（採否）
は行わない（呼び出し側の責務）。

実装は docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3b_matcher/
matcher_cli.py の mast3r 経路（run_mast3r）を移植したもの。移植にあたり query/render
という変数名を imgA/imgB に改名し、`--resolution` オプション（full|1280x720）を追加した
以外のロジック変更はしていない。

実行方法（プロジェクトルートで）:
    uv run --project matcher_lab python matcher_lab/mast3r_cli.py \\
        imgA.png imgB.png --out matches.npz [--resolution full|1280x720]

--resolution: full（既定。原寸のまま MASt3R へ渡す） /
              1280x720（CLI内部で1280x720に縮小してから推論し、マッチ座標を原寸へ逆写像する）
出力 NPZ: u_a, u_b（原寸座標 float64 (N,2)）、conf（float32 (N,)。MASt3R は信頼度を
          返さないため1.0埋め）、meta（JSON文字列: 元解像度・処理解像度・縮小スケール係数・
          逆写像round-trip最大誤差[px]・原寸範囲内率・処理時間[s]）
終了コード: 0=成功（マッチ0点でも成功） / 42=CUDA OOM / 1=その他エラー
            （画像不正・原寸以外のサイズ・チェックポイント不在）
"""
import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import torch

MAST3R_ROOT = Path.home() / "git" / "mast3r"
MAST3R_CKPT = (Path.home() / "data" / "models" / "mast3r"
               / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")

ORIG_W, ORIG_H = 1920, 1080  # design §2.1: 入力は原寸1920x1080で固定
FALLBACK_W, FALLBACK_H = 1280, 720  # design §2.1: --resolution 1280x720
GRID_N = 5  # round-trip検証グリッド 5x5=25点


def roundtrip_and_range(fwd, inv, u_orig_list) -> dict:
    """逆写像の検証値（round-trip最大誤差・原寸範囲内率）を計算する。

    fwd: 原寸→MASt3R内部座標, inv: MASt3R内部座標→原寸。
    u_orig_list: 逆写像済みマッチ座標（原寸）のリスト（範囲内率の計算対象）。
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


def run_mast3r(img_a_path: str, img_b_path: str, resolution: str, device) -> tuple:
    """MASt3R。戻り値: (u_a, u_b, conf=1埋め, meta)。座標は原寸。

    load_images(size=512) + fast_reciprocal_NNs（README既定値）。
    --resolution 1280x720 の場合は、原寸画像を先に1280x720へ縮小した一時PNGを作り、
    そのパスを load_images に渡す（処理解像度→MASt3R内部解像度512への2段リサイズ）。
    逆写像は「原寸↔処理解像度」「処理解像度↔MASt3R内部解像度」の2つの写像を合成して求める。
    """
    sys.path.insert(0, str(MAST3R_ROOT))
    sys.path.insert(0, str(MAST3R_ROOT / "dust3r"))
    from mast3r.model import AsymmetricMASt3R
    from mast3r.fast_nn import fast_reciprocal_NNs
    from dust3r.inference import inference
    from dust3r.utils.image import load_images

    if resolution == "full":
        proc_w, proc_h = ORIG_W, ORIG_H
        scale_o2p = 1.0
        load_paths = [img_a_path, img_b_path]
        tmp_ctx = None
    else:  # 1280x720
        proc_w, proc_h = FALLBACK_W, FALLBACK_H
        scale_o2p = FALLBACK_W / ORIG_W  # = 1/1.5（縦横比が同一のため単一係数で表せる）
        tmp_ctx = tempfile.TemporaryDirectory()
        load_paths = []
        for name, src_path in (("a", img_a_path), ("b", img_b_path)):
            img_bgr = cv2.imread(src_path, cv2.IMREAD_COLOR)
            resized = cv2.resize(img_bgr, (FALLBACK_W, FALLBACK_H), interpolation=cv2.INTER_AREA)
            dst_path = str(Path(tmp_ctx.name) / f"{name}.png")
            cv2.imwrite(dst_path, resized)
            load_paths.append(dst_path)

    # 「原寸→処理解像度」の写像（画素中心規約）: proc = (orig+0.5)*scale_o2p - 0.5
    def fwd_o2p(u):
        return (u + 0.5) * scale_o2p - 0.5

    def inv_o2p(u):
        return (u + 0.5) / scale_o2p - 0.5

    try:
        model = AsymmetricMASt3R.from_pretrained(str(MAST3R_CKPT)).to(device).eval()
        images = load_images(load_paths, size=512, verbose=False)
        with torch.inference_mode():
            output = inference([tuple(images)], model, device, batch_size=1, verbose=False)
        desc1 = output["pred1"]["desc"].squeeze(0).detach()
        desc2 = output["pred2"]["desc"].squeeze(0).detach()
        m0, m1 = fast_reciprocal_NNs(desc1, desc2, subsample_or_initxy1=8,
                                     device=device, dist="dot", block_size=2**13)

        # 逆写像式の導出: load_images は長辺512へリサイズ(round)後、中心で16の倍数へ切り詰め
        h2, w2 = (int(v) for v in images[0]["true_shape"].flatten())
        s = 512 / max(proc_w, proc_h)
        wr, hr = round(proc_w * s), round(proc_h * s)  # リサイズ直後(切り詰め前)
        ox, oy = (wr - w2) // 2, (hr - h2) // 2         # 中心切り詰めのオフセット
        sx, sy = wr / proc_w, hr / proc_h

        # 「処理解像度→MASt3R内部解像度」の写像
        def fwd_p2i(u):
            return np.stack([(u[:, 0] + 0.5) * sx - 0.5 - ox,
                             (u[:, 1] + 0.5) * sy - 0.5 - oy], axis=1)

        def inv_p2i(u):
            return np.stack([(u[:, 0] + ox + 0.5) / sx - 0.5,
                             (u[:, 1] + oy + 0.5) / sy - 0.5], axis=1)

        # 合成写像（原寸 ←→ MASt3R内部座標）
        def fwd(u):
            return fwd_p2i(fwd_o2p(u))

        def inv(u):
            return inv_o2p(inv_p2i(u))

        u_a = inv(np.asarray(m0, dtype=np.float64).reshape(-1, 2))
        u_b = inv(np.asarray(m1, dtype=np.float64).reshape(-1, 2))
        conf = np.ones(len(u_a), dtype=np.float32)
        meta = {"matcher": "mast3r", "orig_resolution": f"{ORIG_W}x{ORIG_H}",
                "resolution": resolution, "proc_resolution": f"{proc_w}x{proc_h}",
                "scale": scale_o2p, "internal_resolution": f"{w2}x{h2}",
                "resized_precrop": [hr, wr], "offsets": [ox, oy], "scale_xy": [sx, sy]}
        meta.update(roundtrip_and_range(fwd, inv, np.vstack([u_a, u_b]) if len(u_a) else []))
        return u_a, u_b, conf, meta
    finally:
        if tmp_ctx is not None:
            tmp_ctx.cleanup()


def main() -> int:
    ap = argparse.ArgumentParser(description="MASt3R ペアマッチングCLI（feat-035）")
    ap.add_argument("imgA", help="画像A（PNG、原寸1920x1080）")
    ap.add_argument("imgB", help="画像B（PNG、原寸1920x1080）")
    ap.add_argument("--out", required=True, help="出力NPZパス")
    ap.add_argument("--resolution", default="full", choices=["full", "1280x720"],
                    help="処理解像度（既定full。1280x720はOOM時のフォールバック用）")
    args = ap.parse_args()

    img_a = cv2.imread(args.imgA, cv2.IMREAD_COLOR)
    img_b = cv2.imread(args.imgB, cv2.IMREAD_COLOR)
    if img_a is None or img_b is None:
        print(f"画像が読めません: {args.imgA} / {args.imgB}", file=sys.stderr)
        return 1
    for name, im in (("imgA", img_a), ("imgB", img_b)):
        if im.shape[:2] != (ORIG_H, ORIG_W):
            print(f"{name} のサイズ {im.shape[:2]} が原寸 ({ORIG_H},{ORIG_W}) と不一致です",
                  file=sys.stderr)
            return 1

    if not MAST3R_CKPT.exists():
        print(f"MASt3Rチェックポイントが見つかりません: {MAST3R_CKPT}", file=sys.stderr)
        return 1

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    t0 = time.time()
    try:
        u_a, u_b, conf, meta = run_mast3r(args.imgA, args.imgB, args.resolution, device)
    except torch.cuda.OutOfMemoryError as e:
        print(f"CUDA OOM: {e}", file=sys.stderr)
        return 42
    meta["elapsed_sec"] = time.time() - t0

    np.savez(args.out, u_a=u_a, u_b=u_b, conf=conf, meta=json.dumps(meta))
    print(f"mast3r: {len(u_a)} マッチ / meta={json.dumps(meta)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
