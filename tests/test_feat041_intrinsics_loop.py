"""feat-041: 内部パラメータ推定ループ関連の単体テスト（GPU・実データ不要）

design.md §8 のテスト表（1〜21、6b、7b、7c、7d、14b。22 は存在しない）に対応する。
prepare_phone_photo / estimate_intrinsics_loop / evaluate_overlay はいずれも torch / gsplat /
Pillow を関数内で遅延 import するため、ルート環境から import できる。
"""
import math
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import estimate_intrinsics_loop as eil
import evaluate_overlay as eov
import prepare_phone_photo as ppp
from estimate_intrinsics_loop import LoopFailure, Params
from render_keypoints import load_cameras_toml, select_camera


# ------------------------------------------------------------
# 共通ヘルパ
# ------------------------------------------------------------

def _params(f=1280.0, cx=960.0, cy=540.0, k1=0.0, k2=0.0, rvec=(0, 0, 0), tvec=(0, 0, 0)):
    return Params(f, cx, cy, k1, k2, np.array(rvec, dtype=np.float64),
                  np.array(tvec, dtype=np.float64))


# ------------------------------------------------------------
# 1〜4 prepare_phone_photo
# ------------------------------------------------------------

def test_1_compute_crop():
    assert ppp.compute_crop(4000, 3000) == (375, 2250)
    assert ppp.compute_crop(4000, 2252) == (1, 2250)
    assert ppp.compute_crop(1920, 1080) == (0, 1080)
    with pytest.raises(ValueError):
        ppp.compute_crop(4000, 2000)
    with pytest.raises(ValueError):
        ppp.compute_crop(4001, 3000)


def test_2_initial_focal_px():
    f = ppp.initial_focal_px(23, 4000, 3000)
    assert abs(f - 23 * 5000 / 43.27 * 0.48) < 1e-9


def test_3_convert_point_roundtrip():
    y0, scale = 375, 1920 / 4000
    for ox, oy in [(0, 0), (1919, 0), (0, 1079), (1919, 1079)]:
        # 出力側の四隅の画素中心に対応する原本座標
        X = (ox + 0.5) / scale - 0.5
        Y = (oy + 0.5) / scale - 0.5 + y0
        x, y = ppp.convert_point(X, Y, y0, scale)
        assert abs(x - ox) < 1e-9 and abs(y - oy) < 1e-9
    # 原本の切り抜き範囲の四隅の画素中心（-0.5 補正込み）
    x, y = ppp.convert_point(0, 375, y0, scale)
    assert abs(x - (0.5 * scale - 0.5)) < 1e-12 and abs(y - (0.5 * scale - 0.5)) < 1e-12


def test_4_validate_exif():
    ok = {"Model": "Galaxy S25", "Orientation": 1, "FocalLength": 5.4,
          "FocalLengthIn35mmFilm": 24, "DigitalZoomRatio": 1.0}
    assert ppp.validate_exif(ok, 4000, 3000, "Galaxy S25", 5.4) == []
    assert ppp.validate_exif({**ok, "Orientation": None}, 4000, 3000, None, None) == []
    assert ppp.validate_exif({**ok, "DigitalZoomRatio": None}, 4000, 3000, None, None) == []
    cases = [
        ({**ok, "Orientation": 6}, 4000, 3000, None, None),
        (ok, 3000, 4000, None, None),
        ({**ok, "FocalLengthIn35mmFilm": None}, 4000, 3000, None, None),
        ({**ok, "FocalLengthIn35mmFilm": 0}, 4000, 3000, None, None),
        ({**ok, "DigitalZoomRatio": 2.0}, 4000, 3000, None, None),
        (ok, 4000, 3000, "Pixel 9", None),
        (ok, 4000, 3000, None, 6.0),
        ({**ok, "FocalLength": None}, 4000, 3000, None, 5.4),
    ]
    for exif, w, h, m, fm in cases:
        assert len(ppp.validate_exif(exif, w, h, m, fm)) == 1, (exif, w, h, m, fm)


# ------------------------------------------------------------
# 5 TOML の読み戻し
# ------------------------------------------------------------

def test_5_toml_roundtrip(tmp_path):
    p0 = tmp_path / "init.toml"
    p0.write_text(ppp.format_init_toml("camA", 1275.7), encoding="utf-8")
    cam = select_camera(load_cameras_toml(str(p0)), "camA")
    assert cam["width"] == 1920 and cam["height"] == 1080
    assert np.allclose(cam["K"], [[1275.7, 0, 960], [0, 1275.7, 540], [0, 0, 1]])
    assert np.allclose(cam["D"], 0) and len(cam["D"]) == 4
    assert np.allclose(cam["rvec"], 0) and np.allclose(cam["tvec"], 0)

    p = _params(1283.4, 957.9, 543.0, 0.0123, -0.0451, (0.1, -0.2, 0.3), (1.0, 2.0, 3.0))
    p1 = tmp_path / "loop.toml"
    p1.write_text(eil.format_loop_toml("camA", p), encoding="utf-8")
    cam = select_camera(load_cameras_toml(str(p1)), "camA")
    assert np.allclose(cam["K"], p.K(), atol=1e-12)
    assert np.allclose(cam["D"], [0.0123, -0.0451, 0, 0], atol=1e-12)
    assert np.allclose(cam["rvec"], p.rvec) and np.allclose(cam["tvec"], p.tvec)
    assert cam["width"] == 1920 and cam["height"] == 1080


# ------------------------------------------------------------
# 6, 6b 同時最小二乗
# ------------------------------------------------------------

def _synth(n=2000, seed=0):
    rng = np.random.default_rng(seed)
    true = _params(1280.0, 955.0, 545.0, 0.05, -0.1, (0.1, -0.2, 0.05), (0.3, -0.2, 0.5))
    K, D = true.K(), true.D()
    R, _ = cv2.Rodrigues(true.rvec.reshape(3, 1))
    px = np.stack([rng.uniform(0, 1919, n * 2), rng.uniform(0, 1079, n * 2)], axis=1)
    z = rng.uniform(1.5, 5.0, n * 2)
    nrm = (px - [960.0, 540.0]) / 1280.0
    Xc = np.stack([nrm[:, 0] * z, nrm[:, 1] * z, z], axis=1)
    Xw = (Xc - true.tvec) @ R
    proj, _ = cv2.projectPoints(Xw, true.rvec.reshape(3, 1), true.tvec.reshape(3, 1), K, D)
    proj = proj.reshape(-1, 2)
    ok = (proj[:, 0] >= 0) & (proj[:, 0] <= 1919) & (proj[:, 1] >= 0) & (proj[:, 1] <= 1079)
    Xw, proj = Xw[ok][:n], proj[ok][:n]
    assert len(Xw) == n
    u = proj + rng.normal(0, 0.3, proj.shape)
    return true, Xw, u


def test_6_joint_refine_synthetic():
    true, X, u = _synth()
    rv0 = true.rvec + np.array([0.0087, 0.0, 0.0])
    tv0 = true.tvec + np.array([0.0115, 0.0115, 0.0115])
    x0 = np.concatenate([[1250.0, 960.0, 540.0, 0.0, 0.0], rv0, tv0])
    new, _status = eil.joint_refine(X, u, x0)
    assert abs(new.f - true.f) / true.f <= 0.005
    assert abs(new.cx - true.cx) <= 3 and abs(new.cy - true.cy) <= 3
    assert abs(new.k1 - true.k1) <= 0.02 and abs(new.k2 - true.k2) <= 0.02


def test_6b_residual_shape():
    rng = np.random.default_rng(1)
    X_w = rng.normal(size=(400, 3)) + [0, 0, 3]
    uq = rng.uniform(0, 1000, (400, 2))
    x = np.array([1280.0, 960.0, 540.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0])
    X = X_w[:300]
    u = uq[:300]
    assert eil._residuals(x, X, u).shape == (600,)
    inl = np.arange(300).reshape(300, 1)         # solvePnPRansac 形式 (N,1)
    inl = np.asarray(inl).flatten()
    X2 = X_w[inl].reshape(-1, 3)
    u2 = uq[inl].reshape(-1, 2)
    assert X2.shape == (300, 3) and u2.shape == (300, 2)
    assert eil._residuals(x, X2, u2).shape == (600,)
    # 形が違う入力は assert で止まる
    with pytest.raises(AssertionError):
        eil.joint_refine(X, uq[:299], x)


# ------------------------------------------------------------
# 7 delta_intr
# ------------------------------------------------------------

def test_7_delta_intr():
    a = _params(1280, 960, 540)
    assert eil.delta_intr(a, _params(1280, 960, 540)) < 1e-9
    b = _params(1280, 960, 540, 0.2, 0.1)
    assert eil.delta_intr(b, _params(1280, 960, 540, 0.2, 0.1)) < 1e-9
    c = eil.delta_intr(a, _params(1281.28, 960, 540))
    assert abs(c - 0.001 * math.hypot(960, 540)) < 0.001


# ------------------------------------------------------------
# 7b, 14b run_loop_iteration（レンダ・MASt3R はモック）
# ------------------------------------------------------------

def _mock_env(monkeypatch, u_q, u_r, N, d_value=3.0):
    monkeypatch.setattr(eil, "render_image",
                        lambda *a, **k: np.zeros((1080, 1920, 3), np.uint8))
    monkeypatch.setattr(eil, "render_depth_alpha_distorted",
                        lambda *a, **k: (np.full((1080, 1920), 3.0, np.float32),
                                         np.ones((1080, 1920), np.float32)))
    monkeypatch.setattr(eil, "call_mast3r",
                        lambda *a, **k: (u_q, u_r, {"in_range_rate": 1.0, "roundtrip_max_px": 0.0}))

    def fake_gate(u_q_, u_r_, depth, alpha, var_map, tau_d, tau_px):
        keep = np.zeros(len(u_q_), dtype=bool)
        keep[:N] = True
        return {"N": int(N), "keep": keep, "d_bilin": np.full(len(u_q_), d_value)}

    monkeypatch.setattr(eil, "gate_breakdown", fake_gate)


@pytest.mark.parametrize("n_gate", [0, 3, 299])
def test_7b_gate_failure(monkeypatch, tmp_path, n_gate):
    rng = np.random.default_rng(0)
    u = rng.uniform(100, 1800, (400, 2))
    _mock_env(monkeypatch, u, u.copy(), n_gate)
    with pytest.raises(LoopFailure) as ei:
        eil.run_loop_iteration(None, _params(), tmp_path / "q.png", 0, tmp_path)
    assert "ゲート通過" in str(ei.value)


def test_14b_point_sync(monkeypatch, tmp_path):
    # 中心付近 300 点 + 逆歪み誤差の大きい端の 1 点（index 150 に挿入）
    rng = np.random.default_rng(3)
    ang = rng.uniform(0, 2 * np.pi, 300)
    rad = rng.uniform(0, 400, 300)
    pts = np.stack([960 + rad * np.cos(ang), 540 + rad * np.sin(ang)], axis=1)
    ur = np.insert(pts, 150, [1919.0, 1079.0], axis=0)
    uq = ur + np.arange(301)[:, None] * 1e-3
    _mock_env(monkeypatch, uq, ur, 301)

    recorded = {}

    def fake_ransac(X, u, K, D, **kw):
        recorded["X"], recorded["u"] = X.copy(), u.copy()
        n = len(X)
        return True, np.zeros((3, 1)), np.zeros((3, 1)), np.arange(n).reshape(n, 1)

    monkeypatch.setattr(eil.cv2, "solvePnPRansac", fake_ransac)
    params = _params(k1=-0.5)
    res = eil.run_loop_iteration(None, params, tmp_path / "q.png", 0, tmp_path)
    assert res["ok"]
    assert recorded["X"].shape == (300, 3) and recorded["u"].shape == (300, 2)
    exp_idx = np.delete(np.arange(301), 150)
    assert np.allclose(recorded["u"], uq[exp_idx])
    proj, _ = cv2.projectPoints(recorded["X"], np.zeros(3), np.zeros(3), params.K(), params.D())
    assert np.max(np.linalg.norm(proj.reshape(-1, 2) - ur[exp_idx], axis=1)) < 0.01
    assert res["diag"]["excl_undist"] == 1


# ------------------------------------------------------------
# 7c 出力済み検査
# ------------------------------------------------------------

def test_7c_existing_output_guard(tmp_path):
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    rep = out_dir / "camA_loop_report.txt"
    rep.write_text("old", encoding="utf-8")
    code = eil.main(["--init-toml", "x.toml", "--camera", "camA", "--ply", "x.ply",
                     "--images-dir", str(tmp_path), "--points-3d", "a.csv",
                     "--points-2d", "b.csv", "--out-dir", str(out_dir)])
    assert code == 1
    assert rep.read_text(encoding="utf-8") == "old"
    assert [p.name for p in out_dir.iterdir()] == ["camA_loop_report.txt"]


# ------------------------------------------------------------
# 7d 前処理の PNG 出力
# ------------------------------------------------------------

def test_7d_prepare_png(monkeypatch, tmp_path):
    img = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    src = tmp_path / "orig.jpg"
    cv2.imwrite(str(src), img)
    monkeypatch.setattr(ppp, "read_exif", lambda path: {
        "Model": "Galaxy S25", "Orientation": 1, "FocalLength": 5.4,
        "FocalLengthIn35mmFilm": 24, "DigitalZoomRatio": 1.0, "Size": (64, 48)})
    csv_in = tmp_path / "p_in.csv"
    lines = ["ObjectName,camera_name,X,Y"]
    for k in range(6):
        lines.append(f"P{k},camA,{10 + k * 8},{12 + k * 4}")
    lines.append("OUT,camA,10,0")          # 切り抜き範囲外（y0=6 より上）
    lines.append("OTHER,camB,10,12")       # 別カメラ
    csv_in.write_text("\n".join(lines) + "\n", encoding="utf-8")
    images_dir = tmp_path / "images"
    out_toml = tmp_path / "init.toml"
    csv_out = tmp_path / "p_out.csv"
    code = ppp.main(["--image", str(src), "--camera", "camA", "--images-dir", str(images_dir),
                     "--init-toml", str(out_toml), "--points-2d-original", str(csv_in),
                     "--points-2d", str(csv_out), "--expect-model", "Galaxy S25",
                     "--expect-focal-mm", "5.4"])
    assert code == 0
    out = cv2.imread(str(images_dir / "camA.png"))
    assert out is not None and out.shape[:2] == (1080, 1920)
    assert not (images_dir / "camA.png.tmp").exists()
    assert not Path(str(out_toml) + ".tmp").exists()
    assert not Path(str(csv_out) + ".tmp").exists()
    rows = csv_out.read_text(encoding="utf-8").splitlines()
    assert rows[0] == "ObjectName,camera_name,X,Y" and len(rows) == 7
    # f = 24 * hypot(64,48)/43.27 * (1920/64)
    cam = select_camera(load_cameras_toml(str(out_toml)), "camA")
    assert abs(cam["K"][0, 0] - 24 * 80 / 43.27 * 30) < 1e-6
    # 出力済みなら --overwrite なしで拒否
    code2 = ppp.main(["--image", str(src), "--camera", "camA", "--images-dir", str(images_dir),
                      "--init-toml", str(out_toml), "--points-2d-original", str(csv_in),
                      "--points-2d", str(csv_out)])
    assert code2 == 1


# ------------------------------------------------------------
# 8, 9, 10
# ------------------------------------------------------------

def test_8_check_ranges():
    assert eil.check_ranges(_params(576.0, 960 + 192.0, 540 + 108.0, 1.0, -1.0)) == []
    assert eil.check_ranges(_params(5760.0, 960 - 192.0, 540 - 108.0)) == []
    assert len(eil.check_ranges(_params(575.99))) == 1
    assert len(eil.check_ranges(_params(5760.01))) == 1
    assert len(eil.check_ranges(_params(1280, 960 + 192.01))) == 1
    assert len(eil.check_ranges(_params(1280, 960 - 192.01))) == 1
    assert len(eil.check_ranges(_params(1280, 960, 540 + 108.01))) == 1
    assert len(eil.check_ranges(_params(1280, 960, 540 - 108.01))) == 1
    assert len(eil.check_ranges(_params(k1=1.01))) == 1
    assert len(eil.check_ranges(_params(k2=-1.01))) == 1
    assert len(eil.check_ranges(_params(100, 0, 0, 2, 2))) == 5


def _cell_points(per_cell, odd_cell=None):
    pts = []
    for cy in range(4):
        for cx in range(4):
            n = per_cell
            if odd_cell == (cx, cy):
                n = per_cell - 1
            for k in range(n):
                pts.append([cx * 480 + 10 + k, cy * 270 + 10 + k])
    return np.array(pts, dtype=np.float64)


def test_9_count_covered_cells():
    assert eil.count_covered_cells(_cell_points(10)) == 16
    assert eil.count_covered_cells(_cell_points(10, odd_cell=(2, 1))) == 15


def test_10_refine_accepted():
    assert eil.refine_accepted("x\n  cam: 受理(単峰)\n", "cam")
    assert eil.refine_accepted("x\n  cam: 受理(仲裁)\n", "cam")
    assert not eil.refine_accepted("x\n  cam: 不受理\n", "cam")
    assert not eil.refine_accepted("x\n  cam: 失敗(入力検証)\n", "cam")


# ------------------------------------------------------------
# 11〜13 evaluate_overlay
# ------------------------------------------------------------

def _texture(seed=0, h=1080, w=1920):
    rng = np.random.default_rng(seed)
    n = rng.normal(size=(h, w)).astype(np.float32)
    n = cv2.GaussianBlur(n, (0, 0), 1.5)
    n = (n - n.mean()) / n.std()
    g = np.clip(128 + 45 * n, 0, 255).astype(np.uint8)
    return cv2.merge([g, g, g])


def test_11_flow_shift_2px():
    photo = _texture()
    M = np.float32([[1, 0, 2], [0, 1, 0]])
    render = cv2.warpAffine(photo, M, (1920, 1080), borderMode=cv2.BORDER_REPLICATE)
    st = eov.flow_stats(photo, render, np.ones((1080, 1920), bool))
    assert st["valid_ratio"] > 0.10
    assert 1.8 <= st["median"] <= 2.2


def test_12_flow_identical():
    photo = _texture(seed=1)
    st = eov.flow_stats(photo, photo.copy(), np.ones((1080, 1920), bool))
    assert st["valid_ratio"] > 0.10
    assert st["median"] <= 0.1


def test_13_judge():
    cases = [
        ((0.09, 0.0, 0.0), "判定不能"),
        ((0.5, 1.0, 3.0), "合格"),
        ((0.5, 1.01, 3.0), "不合格"),
        ((0.5, 1.0, 50.0), "合格"),
        ((0.5, 1.0004, 3.0), "不合格"),
        ((0.5, 0.9996, 3.0), "合格"),
        ((0.0999, 0.5, 3.0), "判定不能"),
        ((0.1, 0.5, 3.0), "合格"),
    ]
    for (vr, med, p95), exp in cases:
        assert eov.judge({"valid_ratio": vr, "median": med, "p95": p95}) == exp


# ------------------------------------------------------------
# 14 lift_to_world、15 refine_undistort_error_px
# ------------------------------------------------------------

@pytest.mark.parametrize("k1,k2", [(0.0, 0.0), (-0.3, 0.0), (0.2, 0.1)])
@pytest.mark.parametrize("pose", ["zero", "arb"])
def test_14_lift_to_world(k1, k2, pose):
    if pose == "zero":
        rvec, tvec = (0, 0, 0), (0, 0, 0)
    else:
        rvec, tvec = (0.1, 0.2, -0.1), (0.1, 0.2, 0.3)
    p = _params(1280, 960, 540, k1, k2, rvec, tvec)
    R, _ = cv2.Rodrigues(p.rvec.reshape(3, 1))
    rng = np.random.default_rng(5)
    px = np.stack([rng.uniform(50, 1870, 200), rng.uniform(50, 1030, 200)], axis=1)
    z = rng.uniform(1.0, 6.0, 200)
    nrm = (px - [960.0, 540.0]) / 1280.0
    # 3D 点 (1.6, 0.8, 2.0) m（カメラ座標）を含める
    Xc = np.vstack([np.stack([nrm[:, 0] * z, nrm[:, 1] * z, z], axis=1), [[1.6, 0.8, 2.0]]])
    Xw = (Xc - p.tvec) @ R
    proj, _ = cv2.projectPoints(Xw, p.rvec.reshape(3, 1), p.tvec.reshape(3, 1), p.K(), p.D())
    X_rec, ok = eil.lift_to_world(proj.reshape(-1, 2), Xc[:, 2], p)
    assert ok.all()
    assert np.max(np.abs(X_rec - Xw)) < 1e-6


def test_15_refine_undistort_error_px():
    K = _params(1280, 960, 540).K()
    assert eil.refine_undistort_error_px(K, np.zeros(4)) <= 1e-6
    assert eil.refine_undistort_error_px(K, np.array([0.05, -0.1, 0, 0])) <= 0.1
    assert eil.refine_undistort_error_px(K, np.array([-0.3, 0.0, 0, 0])) > 0.1


# ------------------------------------------------------------
# 16, 17, 18 マッチャー切り替え・LoFTR OOM・収束判定
# ------------------------------------------------------------

def _central_points(n=300, seed=3):
    rng = np.random.default_rng(seed)
    ang = rng.uniform(0, 2 * np.pi, n)
    rad = rng.uniform(0, 400, n)
    return np.stack([960 + rad * np.cos(ang), 540 + rad * np.sin(ang)], axis=1)


@pytest.mark.parametrize("i", [0, 1, 2])
def test_16_matcher_switch(monkeypatch, tmp_path, i):
    pts = _central_points(300)
    calls = {"mast3r": 0, "loftr": 0, "gate_n": None}
    meta = {"in_range_rate": 1.0, "roundtrip_max_px": 0.0}
    # LoFTR 側: 信頼度 0.5 が 300 件、0.1 が 50 件（先頭に混ぜる）
    junk = _central_points(50, seed=9)
    u_all = np.vstack([junk, pts])
    conf = np.concatenate([np.full(50, 0.1), np.full(300, 0.5)])

    _mock_env(monkeypatch, pts, pts.copy(), 300)

    def fake_mast3r(*a, **k):
        calls["mast3r"] += 1
        return pts, pts.copy(), meta

    def fake_matcher(*a, **k):
        calls["loftr"] += 1
        return u_all, u_all.copy(), conf, meta

    def fake_gate(u_q_, u_r_, depth, alpha, var_map, tau_d, tau_px):
        calls["gate_n"] = len(u_q_)
        return {"N": len(u_q_), "keep": np.ones(len(u_q_), dtype=bool),
                "d_bilin": np.full(len(u_q_), 3.0)}

    def fake_ransac(X, u, K, D, **kw):
        n = len(X)
        return True, np.zeros((3, 1)), np.zeros((3, 1)), np.arange(n).reshape(n, 1)

    monkeypatch.setattr(eil, "call_mast3r", fake_mast3r)
    monkeypatch.setattr(eil, "call_matcher", fake_matcher)
    monkeypatch.setattr(eil, "gate_breakdown", fake_gate)
    monkeypatch.setattr(eil.cv2, "solvePnPRansac", fake_ransac)
    res = eil.run_loop_iteration(None, _params(), tmp_path / "q.png", i, tmp_path)
    if i == 0:
        assert calls["mast3r"] == 1 and calls["loftr"] == 0
        assert res["diag"]["matcher"] == "MASt3R"
    else:
        assert calls["mast3r"] == 0 and calls["loftr"] == 1
        assert res["diag"]["matcher"] == "LoFTR"
        assert calls["gate_n"] == 300
        assert res["diag"]["matches"] == 300


def test_17_loftr_oom(monkeypatch, tmp_path):
    _mock_env(monkeypatch, np.zeros((10, 2)), np.zeros((10, 2)), 10)

    def oom(*a, **k):
        raise eil.MatcherOOM()

    monkeypatch.setattr(eil, "call_matcher", oom)
    with pytest.raises(LoopFailure) as ei:
        eil.run_loop_iteration(None, _params(), tmp_path / "q.png", 1, tmp_path)
    assert "LoFTR OOM" in str(ei.value)


def test_18_is_converged():
    cases = [((0, 0.0), False), ((0, 1.99), False), ((1, 1.99), True),
             ((1, 2.0), False), ((2, 0.0), True)]
    for (i, d), exp in cases:
        assert eil.is_converged(i, d) is exp


def test_19_not_converged_writes_loop_toml(monkeypatch, tmp_path):
    camera = "camA"
    K0 = np.array([[1280.0, 0, 960.0], [0, 1280.0, 540.0], [0, 0, 1]])
    cam0 = {"K": K0, "D": np.zeros(4), "width": 1920, "height": 1080,
            "rvec": np.zeros(3), "tvec": np.zeros(3)}
    img_path = tmp_path / "camA.png"

    def mock_iter_params(i):
        return Params(1280.0 + 20.0 * (i % 2) + 0.1 * i, 960.0 + 0.01 * i, 540.0 - 0.01 * i,
                      0.001 * i, -0.001 * i,
                      np.array([0.01 * i, 0.02, 0.03]), np.array([0.1 * i, 0.2, 0.3]))

    calls = []

    def mock_run_loop_iteration(gaussians, params, image_path, i, tmp_dir):
        calls.append(i)
        diag = {"matcher": "LoFTR", "matches": 1000, "N": 800, "inl": 700, "rmse": 1.0,
                "cover": 16, "lsq_status": 1, "elapsed": 0.1}
        return {"ok": True, "params": mock_iter_params(i), "diag": diag}

    monkeypatch.setattr(eil, "load_cameras_toml", lambda path: {camera: cam0})
    monkeypatch.setattr(eil, "select_camera", lambda cams, name: cams[name])
    monkeypatch.setattr(eil, "load_points_3d_csv", lambda path: {})
    monkeypatch.setattr(eil, "load_points_2d_csv", lambda path: {})
    monkeypatch.setattr(eil, "validate_camera", lambda *a, **k: {
        "ok": True, "p3": np.zeros((6, 3)), "p2": np.zeros((6, 2)),
        "names": list("abcdef"), "image_path": img_path})
    monkeypatch.setattr(eil, "pnp_manual",
                        lambda *a, **k: (np.zeros(3), np.zeros(3), np.arange(6)))
    monkeypatch.setattr(eil, "median_reproj_px", lambda *a, **k: 0.0)
    monkeypatch.setattr(eil, "run_loop_iteration", mock_run_loop_iteration)
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: object()
    monkeypatch.setitem(sys.modules, "render", dummy)

    out_dir = tmp_path / "out"
    code = eil.main(["--init-toml", "x.toml", "--camera", camera, "--ply", "x.ply",
                     "--images-dir", str(tmp_path), "--points-3d", "a.csv",
                     "--points-2d", "b.csv", "--out-dir", str(out_dir),
                     "--tmp-dir", str(tmp_path / "tmp")])
    assert code == 3
    assert calls == list(range(8))
    loop_toml = out_dir / f"{camera}_loop.toml"
    assert loop_toml.exists()
    cam = select_camera(load_cameras_toml(str(loop_toml)), camera)
    exp = mock_iter_params(7)
    assert abs(cam["K"][0, 0] - exp.f) < 1e-9
    assert abs(cam["K"][0, 2] - exp.cx) < 1e-9 and abs(cam["K"][1, 2] - exp.cy) < 1e-9
    assert abs(cam["D"][0] - exp.k1) < 1e-9 and abs(cam["D"][1] - exp.k2) < 1e-9
    assert np.allclose(cam["rvec"], exp.rvec, atol=1e-9)
    assert np.allclose(cam["tvec"], exp.tvec, atol=1e-9)
    other = mock_iter_params(5)
    assert abs(cam["K"][0, 0] - other.f) > 1e-3
    assert not (out_dir / f"{camera}_final.toml").exists()
    for pth in eov.eval_output_paths(str(out_dir / f"{camera}_eval")).values():
        assert not Path(pth).exists()
    report = (out_dir / f"{camera}_loop_report.txt").read_text(encoding="utf-8")
    assert "未収束（ループ結果 TOML は最後の反復の値）" in report


def test_20_overwrite_removed_and_dangling_link(tmp_path):
    base = ["--init-toml", "x.toml", "--camera", "camA", "--ply", "x.ply",
            "--images-dir", str(tmp_path), "--points-3d", "a.csv", "--points-2d", "b.csv"]
    # (a) --overwrite は引数エラー
    with pytest.raises(SystemExit) as ei:
        eil.main(base + ["--out-dir", str(tmp_path / "outA"), "--overwrite"])
    assert ei.value.code == 2
    # (b) リンク先の無いシンボリックリンクも出力済みとして拒否
    out_dir = tmp_path / "outB"
    out_dir.mkdir()
    target = tmp_path / "nowhere" / "target.toml"
    link = out_dir / "camA_final.toml"
    link.symlink_to(target)
    code = eil.main(base + ["--out-dir", str(out_dir)])
    assert code == 1
    assert [p.name for p in out_dir.iterdir()] == ["camA_final.toml"]
    assert link.is_symlink()
    assert not target.exists() and not target.parent.exists()


def _bug006_setup(monkeypatch, tmp_path, camera="camA"):
    """テスト 19 と同じモック。run_loop_iteration の呼び出し履歴を返す。"""
    K0 = np.array([[1280.0, 0, 960.0], [0, 1280.0, 540.0], [0, 0, 1]])
    cam0 = {"K": K0, "D": np.zeros(4), "width": 1920, "height": 1080,
            "rvec": np.zeros(3), "tvec": np.zeros(3)}
    img_path = tmp_path / "camA.png"
    calls = []

    def mock_params(i):
        return Params(1280.0 + 20.0 * (i % 2) + 0.1 * i, 960.0 + 0.01 * i, 540.0 - 0.01 * i,
                      0.001 * i, -0.001 * i,
                      np.array([0.01 * i, 0.02, 0.03]), np.array([0.1 * i, 0.2, 0.3]))

    def mock_run_loop_iteration(gaussians, params, image_path, i, tmp_dir):
        calls.append(i)
        diag = {"matcher": "LoFTR", "matches": 1000, "N": 800, "inl": 700, "rmse": 1.0,
                "cover": 16, "lsq_status": 1, "elapsed": 0.1}
        return {"ok": True, "params": mock_params(i), "diag": diag}

    monkeypatch.setattr(eil, "load_cameras_toml", lambda path: {camera: cam0})
    monkeypatch.setattr(eil, "select_camera", lambda cams, name: cams[name])
    monkeypatch.setattr(eil, "load_points_3d_csv", lambda path: {})
    monkeypatch.setattr(eil, "load_points_2d_csv", lambda path: {})
    monkeypatch.setattr(eil, "validate_camera", lambda *a, **k: {
        "ok": True, "p3": np.zeros((6, 3)), "p2": np.zeros((6, 2)),
        "names": list("abcdef"), "image_path": img_path})
    monkeypatch.setattr(eil, "pnp_manual",
                        lambda *a, **k: (np.zeros(3), np.zeros(3), np.arange(6)))
    monkeypatch.setattr(eil, "median_reproj_px", lambda *a, **k: 0.0)
    monkeypatch.setattr(eil, "run_loop_iteration", mock_run_loop_iteration)
    dummy = types.ModuleType("render")
    dummy.load_ply = lambda path: object()
    monkeypatch.setitem(sys.modules, "render", dummy)
    return calls


def _bug006_args(tmp_path, camera, out_dir, tmp_dir=None):
    a = ["--init-toml", "x.toml", "--camera", camera, "--ply", "x.ply",
         "--images-dir", str(tmp_path), "--points-3d", "a.csv",
         "--points-2d", "b.csv", "--out-dir", str(out_dir)]
    if tmp_dir is not None:
        a += ["--tmp-dir", str(tmp_dir)]
    return a


def test_23_out_dir_must_be_new(monkeypatch, tmp_path):
    calls = _bug006_setup(monkeypatch, tmp_path)
    # (a) 既存の空ディレクトリ
    out_a = tmp_path / "outA"
    out_a.mkdir()
    assert eil.main(_bug006_args(tmp_path, "camA", out_a)) == 1
    assert list(out_a.iterdir()) == []
    # (b) リンク先の無いシンボリックリンク
    out_b = tmp_path / "outB"
    out_b.symlink_to(tmp_path / "nowhere")
    assert eil.main(_bug006_args(tmp_path, "camA", out_b)) == 1
    assert out_b.is_symlink() and not (tmp_path / "nowhere").exists()
    # (c) 未作成の親と .. を含み既存ディレクトリを指すパス
    outx = tmp_path / "outx"
    outx.mkdir()
    old = outx / "camA_final.toml.tmp"
    old.write_text("old", encoding="utf-8")
    assert eil.main(_bug006_args(tmp_path, "camA", tmp_path / "new2" / ".." / "outx")) == 1
    assert old.read_text(encoding="utf-8") == "old"
    assert calls == []


def test_24_tmp_dir_must_be_new(monkeypatch, tmp_path, capsys):
    calls = _bug006_setup(monkeypatch, tmp_path)
    # (a) 既存の空ディレクトリ
    tmp_a = tmp_path / "tmpA"
    tmp_a.mkdir()
    out_a = tmp_path / "outA"
    assert eil.main(_bug006_args(tmp_path, "camA", out_a, tmp_a)) == 1
    assert not out_a.exists() and list(tmp_a.iterdir()) == []
    # (b) リンク先の無いシンボリックリンク
    tmp_b = tmp_path / "tmpB"
    tmp_b.symlink_to(tmp_path / "nowhere")
    out_b = tmp_path / "outB"
    assert eil.main(_bug006_args(tmp_path, "camA", out_b, tmp_b)) == 1
    assert not out_b.exists() and tmp_b.is_symlink()
    assert calls == []
    # (c) 対照: 存在しないパスなら従来どおり進む
    out_c = tmp_path / "outC"
    assert eil.main(_bug006_args(tmp_path, "camA", out_c, tmp_path / "tmpC")) == 3
    assert calls == list(range(8))


def test_25_parent_symlink_reuse(monkeypatch, tmp_path):
    calls = _bug006_setup(monkeypatch, tmp_path)
    existing = tmp_path / "existing"
    (existing / "sub").mkdir(parents=True)
    old = existing / "sub" / "iter00_render.png"
    old.write_text("old", encoding="utf-8")
    (tmp_path / "link").symlink_to("new/../existing")
    out_dir = tmp_path / "new"
    code = eil.main(_bug006_args(tmp_path, "camA", out_dir, tmp_path / "link" / "sub"))
    assert code == 1
    assert old.read_text(encoding="utf-8") == "old"
    assert calls == []
    assert not out_dir.exists()


def test_26_camera_name_restriction(monkeypatch, tmp_path):
    calls = _bug006_setup(monkeypatch, tmp_path)
    victim_report = tmp_path / "victim_loop_report.txt"
    victim_report.write_text("old", encoding="utf-8")
    for i, cam in enumerate([str(tmp_path / "victim"), "sub/cam", ".."]):
        out_dir = tmp_path / f"out26_{i}"
        assert eil.main(_bug006_args(tmp_path, cam, out_dir)) == 1
        assert not out_dir.exists()
    assert victim_report.read_text(encoding="utf-8") == "old"
    assert calls == []


def test_27_tmp_dir_creation_partial_failure(monkeypatch, tmp_path, capsys):
    calls = _bug006_setup(monkeypatch, tmp_path)
    out_dir = tmp_path / "new"
    code = eil.main(_bug006_args(tmp_path, "camA", out_dir, out_dir / "sub" / ".."))
    assert code == 1
    err = capsys.readouterr().err
    assert "中間ファイル置き場が既に存在します" in err
    assert "作成したディレクトリが残っています" in err
    assert calls == []


def test_21_format_eval_txt():
    stats = {"n_valid": 207360, "valid_ratio": 0.1, "median": 1.0004, "p95": 4.0,
             "mean": 1.2, "max": 50.0}
    txt = eov.format_eval_txt(stats, "不合格", "dummy")
    lines = txt.splitlines()
    assert "median: 1.000 px" in lines
    assert "median_raw: 1.0004" in lines
    assert "valid_ratio_raw: 0.1" in lines
    raw = {l.split(": ", 1)[0]: l.split(": ", 1)[1] for l in lines if "_raw: " in l}
    assert float(raw["median_raw"]) == stats["median"]
    assert float(raw["valid_ratio_raw"]) == stats["valid_ratio"]
    assert "合格ライン: median <= 1.000 px（valid_ratio >= 0.100）。p95 は記録のみ" in lines
