"""feat-035: phase4/adjust_extrinsics_multiview.py の単体テスト（Step 2〜6: 本体骨格 + Stage P/B/F/A/D/E）

design.md §4「テスト設計」のうち Step 2〜6 範囲（ペア列挙・採用判定・カメラ辞書マージ規則・
gate_depth_quality・ブートストラップの貪欲順序と確定fail条件・Stage F エピポーラ整合フィルタ・
main() の --init-cameras 全カメラ必須チェック・Stage A の collect_anchor_points 入出力契約・
Stage D のホールドアウト分割/BAProblem/NaNバリア/三角測量/交会角重み/非悪化フォールバック/
BA参加条件境界/合成データBA復元・Stage E の出力TOML往復と出力対象カメラ限定）を検証する。
subprocess・GPUに依存しない純粋関数のみを対象とする。run_pair_matching 自体（mast3r_cli.py
の subprocess 呼び出し部）と、実際のGPUレンダを要する render_depth_alpha_distorted /
render_image / call_matcher の中身は本テスト範囲外とする（design §4 注記どおり、純粋関数
部分を分離してテストする方針。bootstrap_cameras・collect_anchor_points のテストでは
render_depth_alpha_distorted・render_image・call_matcher・cv2.solvePnPRansac を
monkeypatch でモックする）。
"""
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import tomli

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import adjust_extrinsics_multiview as aem
from adjust_extrinsics_multiview import (
    BA_DEGRADE_TOL_PX,
    EPI_TOL_PX,
    N_ANCHOR_MIN,
    N_PAIR_MIN,
    TRI_MIN_ANGLE_DEG,
    BAResult,
    InputError,
    apply_ba_fallback,
    bootstrap_cameras,
    build_ba_problem,
    build_camera_dict,
    build_cross_points,
    crossing_angle_weight,
    enumerate_pairs,
    filter_cross_matches,
    gate_depth_quality,
    is_pair_adopted,
    run_bundle_adjustment,
    select_ba_camera_names,
    split_holdout,
    write_multiview_toml,
)
from render_keypoints import load_cameras_toml


# ========================================
# ペア列挙（design §2.2「対象カメラの全ペア（辞書順）」）
# ========================================

class TestEnumeratePairs:
    def test_dict_order_sorted_regardless_of_input_order(self):
        names = ["cam03", "cam01", "cam02"]
        pairs = enumerate_pairs(names)
        assert pairs == [("cam01", "cam02"), ("cam01", "cam03"), ("cam02", "cam03")]

    def test_pair_count_matches_combination_8_cameras(self):
        names = [f"cam{i:02d}" for i in range(8)]
        pairs = enumerate_pairs(names)
        assert len(pairs) == 28  # C(8,2)
        assert len(set(pairs)) == 28  # 重複なし

    def test_all_pairs_have_camA_less_than_camB(self):
        names = ["camZ", "camA", "camM"]
        pairs = enumerate_pairs(names)
        assert len(pairs) == 3
        for cam_a, cam_b in pairs:
            assert cam_a < cam_b

    def test_duplicate_names_deduplicated(self):
        names = ["cam01", "cam02", "cam01"]
        pairs = enumerate_pairs(names)
        assert pairs == [("cam01", "cam02")]

    def test_single_camera_no_pairs(self):
        assert enumerate_pairs(["cam01"]) == []


# ========================================
# 採用判定境界（design §2.2 手順5、N_PAIR_MIN=50）
# ========================================

class TestIsPairAdopted:
    def test_n_pair_min_is_50(self):
        assert N_PAIR_MIN == 50

    def test_below_threshold_49_rejected(self):
        assert is_pair_adopted(49) is False

    def test_at_threshold_50_adopted(self):
        assert is_pair_adopted(50) is True

    def test_above_threshold_adopted(self):
        assert is_pair_adopted(51) is True

    def test_zero_rejected(self):
        assert is_pair_adopted(0) is False


# ========================================
# カメラ辞書マージ規則（design §2.0）
# ========================================

_TOML_INTRINSICS = """\
[camA]
name = "camA"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.02, 0.001, -0.002]
rotation = [9.0, 9.0, 9.0]
translation = [9.0, 9.0, 9.0]

[camB]
name = "camB"
size = [1920.0, 1080.0]
matrix = [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.04, 0.01, 0.0, 0.0]
rotation = [8.0, 8.0, 8.0]
translation = [8.0, 8.0, 8.0]
"""

# camA分のみ初期ポーズを持つ（camBはブートストラップ対象=Noneになるべき）
_TOML_INIT_POSES = """\
[camA]
name = "camA"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.02, 0.001, -0.002]
rotation = [0.1, 0.2, 0.3]
translation = [0.5, -0.3, 5.0]
"""

# feat-026出力TOML相当: camA/camBの両方にセクションがある（camBは未精緻化の無効値が残る想定）
_TOML_INIT_POSES_ALL = """\
[camA]
name = "camA"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.02, 0.001, -0.002]
rotation = [0.1, 0.2, 0.3]
translation = [0.5, -0.3, 5.0]

[camB]
name = "camB"
size = [1920.0, 1080.0]
matrix = [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.04, 0.01, 0.0, 0.0]
rotation = [99.0, 99.0, 99.0]
translation = [99.0, 99.0, 99.0]
"""


class TestBuildCameraDict:
    def test_intrinsics_taken_from_toml(self, tmp_path):
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES)

        cams = build_camera_dict(str(toml_path), str(init_path))

        assert np.allclose(cams["camA"]["K"],
                           [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        assert np.allclose(cams["camA"]["D"], [-0.05, 0.02, 0.001, -0.002])
        assert cams["camA"]["width"] == 1920
        assert cams["camA"]["height"] == 1080

    def test_toml_rotation_translation_discarded_init_toml_used_instead(self, tmp_path):
        """--tomlのrotation/translationは読み捨てられ、--init-tomlの値がrvec/tvecになる"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES)

        cams = build_camera_dict(str(toml_path), str(init_path))

        # --tomlのrotation=[9,9,9]ではなく--init-tomlのrotation=[0.1,0.2,0.3]が使われる
        assert np.allclose(cams["camA"]["rvec"], [0.1, 0.2, 0.3])
        assert np.allclose(cams["camA"]["tvec"], [0.5, -0.3, 5.0])

    def test_camera_not_in_init_toml_has_none_pose(self, tmp_path):
        """--init-tomlに存在しないカメラはrvec/tvecがNone（Stage Bの対象）"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES)  # camBは含まない

        cams = build_camera_dict(str(toml_path), str(init_path))

        assert cams["camB"]["rvec"] is None
        assert cams["camB"]["tvec"] is None
        # camBの内部パラメータは--tomlから正しく読める
        assert np.allclose(cams["camB"]["K"],
                           [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]])

    def test_all_cameras_none_pose_when_init_toml_not_given(self, tmp_path):
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)

        cams = build_camera_dict(str(toml_path), None)

        assert cams["camA"]["rvec"] is None
        assert cams["camA"]["tvec"] is None
        assert cams["camB"]["rvec"] is None
        assert cams["camB"]["tvec"] is None


# ========================================
# --init-cameras によるマージ規則（design §2.0、FR-002）
# ========================================

class TestInitCameras:
    def test_only_specified_camera_gets_pose_even_if_init_toml_has_all(self, tmp_path):
        """全カメラを含むinit-toml（feat-026出力TOML相当）でも、--init-camerasに
        指定したカメラのみポーズを持ち、それ以外はNoneになる（未精緻化カメラの
        無効ポーズが誤って較正済み扱いされる事象への対策、FR-002）"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES_ALL)  # camA/camB両方にセクションあり

        cams = build_camera_dict(str(toml_path), str(init_path), ["camA"])

        assert np.allclose(cams["camA"]["rvec"], [0.1, 0.2, 0.3])
        assert np.allclose(cams["camA"]["tvec"], [0.5, -0.3, 5.0])
        assert cams["camB"]["rvec"] is None
        assert cams["camB"]["tvec"] is None
        # camBの内部パラメータは--tomlから正しく読める（ポーズだけがNoneになる）
        assert np.allclose(cams["camB"]["K"],
                           [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]])

    def test_camera_not_in_init_toml_raises_input_error(self, tmp_path):
        """--init-camerasに指定したカメラがinit-tomlに存在しない場合はエラー終了"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES)  # camBを含まない

        with pytest.raises(aem.InputError):
            build_camera_dict(str(toml_path), str(init_path), ["camA", "camB"])

    def test_omitted_trusts_all_init_toml_cameras(self, tmp_path):
        """--init-cameras省略時はinit-tomlの全カメラを信頼する（両方ポーズを持つ）"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES_ALL)

        cams = build_camera_dict(str(toml_path), str(init_path), None)

        assert np.allclose(cams["camA"]["rvec"], [0.1, 0.2, 0.3])
        assert np.allclose(cams["camB"]["rvec"], [99.0, 99.0, 99.0])
        assert cams["camA"]["tvec"] is not None
        assert cams["camB"]["tvec"] is not None


# ========================================
# gate_depth_quality（design §2.3-3。ブートストラップ専用ゲート）
# ========================================

def _uniform_map(shape=(21, 21), depth_value=5.0, alpha_value=1.0):
    depth = np.full(shape, depth_value, dtype=np.float64)
    alpha = np.full(shape, alpha_value, dtype=np.float64)
    return depth, alpha


class TestGateDepthQuality:
    def test_baseline_all_conditions_pass(self):
        depth, alpha = _uniform_map()
        uv = np.array([[10.0, 10.0]])
        keep, depths = gate_depth_quality(uv, depth, alpha)
        assert keep[0]
        assert np.isclose(depths[0], 5.0)

    def test_depth_discontinuity_rejects_alone(self):
        """depth_ok違反（2x2近傍にNaN）が単独で reject を引き起こす"""
        depth, alpha = _uniform_map()
        depth[11, 11] = np.nan
        uv = np.array([[10.0, 10.0]])
        keep, _ = gate_depth_quality(uv, depth, alpha)
        assert not keep[0]

    def test_low_alpha_rejects_alone(self):
        """alpha違反が単独で reject を引き起こす（depth/z/varは正常なまま）"""
        depth, alpha = _uniform_map()
        alpha[10, 10] = 0.3  # GATE_ALPHA(0.5)未満
        uv = np.array([[10.0, 10.0]])
        keep, depths = gate_depth_quality(uv, depth, alpha)
        assert not keep[0]
        assert np.isclose(depths[0], 5.0)  # depth自体は正常値のまま

    def test_out_of_z_range_rejects_alone(self):
        """z範囲違反が単独で reject を引き起こす（2x2近傍だけ範囲外だが局所的には滑らか）"""
        depth, alpha = _uniform_map()
        depth[10:12, 10:12] = 20.0  # GATE_Z_RANGE=(0.5, 10.0)の範囲外
        uv = np.array([[10.0, 10.0]])
        keep, depths = gate_depth_quality(uv, depth, alpha)
        assert not keep[0]
        assert np.isclose(depths[0], 20.0)

    def test_high_local_variance_rejects_alone(self):
        """局所分散違反が単独で reject を引き起こす（直近2x2近傍は滑らかなまま）"""
        depth, alpha = _uniform_map()
        depth[8:13, 8:13] = 8.0     # GATE_VAR_WINDOW=5の窓の大半を別深度に
        depth[10:12, 10:12] = 5.0   # 直近2x2近傍は滑らかに保つ（depth_okは通す）
        uv = np.array([[10.0, 10.0]])
        keep, _ = gate_depth_quality(uv, depth, alpha)
        assert not keep[0]

    def test_pixel_valid_empty_rejects_all_without_crash(self):
        """pixel_validが0件の場合は全点rejectとする（median(空配列)によるクラッシュを避ける）"""
        depth, alpha = _uniform_map(alpha_value=0.0)  # 全画素がGATE_ALPHA以下
        uv = np.array([[5.0, 5.0], [10.0, 10.0], [15.0, 15.0]])
        keep, depths = gate_depth_quality(uv, depth, alpha)
        assert keep.shape == (3,)
        assert not keep.any()

    def test_independent_of_pixel_displacement(self):
        """画素変位（2画像間の距離）は判定に使わない: 遠く離れた2点でも深度品質のみで判定される"""
        depth, alpha = _uniform_map()
        uv = np.array([[2.0, 2.0], [18.0, 18.0]])
        keep, _ = gate_depth_quality(uv, depth, alpha)
        assert keep.all()


# ========================================
# bootstrap_cameras（design §2.3。貪欲順序・試行失敗/確定失敗）
# ========================================

def _pair_key(cam_a: str, cam_b: str) -> tuple[str, str]:
    return tuple(sorted([cam_a, cam_b]))


def _make_cam(k_fx: float, rvec=None, tvec=None) -> dict:
    K = np.array([[k_fx, 0.0, 960.0], [0.0, k_fx, 540.0], [0.0, 0.0, 1.0]])
    D = np.zeros(4)
    return {"K": K, "D": D, "width": 1920, "height": 1080, "rvec": rvec, "tvec": tvec}


def _adopted_pair(cam_a: str, cam_b: str, n: int, n_pts: int = 32,
                  coord: tuple = (10.0, 10.0)) -> tuple[tuple, dict]:
    key = _pair_key(cam_a, cam_b)
    pts = np.tile(np.array(coord, dtype=np.float64), (n_pts, 1))
    return key, {"u_a": pts.copy(), "u_b": pts.copy(), "n": n, "status": "adopted"}


class TestBootstrapGreedyOrder:
    def test_processes_max_match_pair_first(self, monkeypatch):
        """design §2.3 手順1: S×Uの採用ペアのうちマッチ数最大のペアから処理される"""
        c0 = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        u1 = _make_cam(1001.0)
        u2 = _make_cam(1002.0)
        u3 = _make_cam(1003.0)
        cams = {"c0": c0, "u1": u1, "u2": u2, "u3": u3}

        pairs = {}
        for name, n in [("u1", 100), ("u2", 80), ("u3", 60)]:
            key, val = _adopted_pair("c0", name, n)
            pairs[key] = val

        depth, alpha = _uniform_map()  # ゲート全通過の一様マップ
        monkeypatch.setattr(aem, "render_depth_alpha_distorted",
                            lambda gaussians, cam, near_plane: (depth, alpha))

        order = []

        def fake_pnp(X_w, uv, K, D, reprojectionError, iterationsCount, flags,
                    useExtrinsicGuess):
            order.append(K[0, 0])  # fxでどのuカメラが処理されたかを識別
            return True, np.zeros((3, 1)), np.zeros((3, 1)), np.arange(len(X_w)).reshape(-1, 1)

        monkeypatch.setattr(cv2, "solvePnPRansac", fake_pnp)

        cams_out, records = bootstrap_cameras(cams, pairs, gaussians={}, images_dir=Path("."))

        assert order == [1001.0, 1002.0, 1003.0]  # n=100,80,60の順（降順）
        assert all(r["status"] == "success" for r in records)
        assert cams_out["u1"]["rvec"] is not None
        assert cams_out["u2"]["rvec"] is not None
        assert cams_out["u3"]["rvec"] is not None


class TestBootstrapFailureHandling:
    def test_trial_failure_does_not_confirm_fail_while_untried_pair_remains(self, monkeypatch):
        """design §2.3 手順5: あるペアで試行失敗しても、別の較正済みカメラとの未試行ペアが
        残っていれば確定失敗にならず、両方を試行し尽くしてから確定失敗になる"""
        c0 = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        c1 = _make_cam(1100.0, rvec=np.zeros(3), tvec=np.array([1.0, 0.0, 0.0]))
        u = _make_cam(1001.0)
        cams = {"c0": c0, "c1": c1, "u": u}

        pairs = {}
        for name, n in [("c0", 100), ("c1", 90)]:
            key, val = _adopted_pair(name, "u", n)
            pairs[key] = val

        # alpha=0の一様マップ→gate_depth_qualityが常に全点rejectし対応点数不足で試行失敗
        bad_depth, bad_alpha = _uniform_map(alpha_value=0.0)
        monkeypatch.setattr(aem, "render_depth_alpha_distorted",
                            lambda gaussians, cam, near_plane: (bad_depth, bad_alpha))

        cams_out, records = bootstrap_cameras(cams, pairs, gaussians={}, images_dir=Path("."))

        assert cams_out["u"]["rvec"] is None
        assert len(records) == 1
        rec = records[0]
        assert rec["camera"] == "u"
        assert rec["status"] == "fail"
        # c0, c1の両方を試行し尽くしてから確定失敗になったことを確認
        assert len(rec["attempts"]) == 2
        assert {a["partner"] for a in rec["attempts"]} == {"c0", "c1"}
        assert all(a["reason"] == "対応点数不足" for a in rec["attempts"])

    def test_confirmed_fail_immediately_when_single_pair_exhausted(self, monkeypatch):
        """未試行ペアが最初から1つしかない場合は、その1回の試行失敗で確定失敗になる"""
        c0 = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        u = _make_cam(1001.0)
        cams = {"c0": c0, "u": u}
        key, val = _adopted_pair("c0", "u", 100)
        pairs = {key: val}

        bad_depth, bad_alpha = _uniform_map(alpha_value=0.0)
        monkeypatch.setattr(aem, "render_depth_alpha_distorted",
                            lambda gaussians, cam, near_plane: (bad_depth, bad_alpha))

        cams_out, records = bootstrap_cameras(cams, pairs, gaussians={}, images_dir=Path("."))

        assert cams_out["u"]["rvec"] is None
        assert len(records) == 1
        assert records[0]["status"] == "fail"
        assert len(records[0]["attempts"]) == 1
        assert records[0]["attempts"][0]["partner"] == "c0"

    def test_no_uncalibrated_cameras_skips_and_returns_empty_records(self):
        """design §2.3 境界条件: Uが最初から空ならStage Bをスキップする"""
        c0 = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        c1 = _make_cam(1100.0, rvec=np.zeros(3), tvec=np.zeros(3))
        cams = {"c0": c0, "c1": c1}
        cams_out, records = bootstrap_cameras(cams, pairs={}, gaussians={}, images_dir=Path("."))
        assert records == []
        assert cams_out is cams


# ========================================
# Stage F: エピポーラ整合フィルタ（design §2.45, FR-010）
#
# 合成カメラは全て rvec=0（回転なし）・tvec=(x_offset,0,0)（横方向並進のみ）に統一する。
# この配置では任意のペアで R_rel=I・t_rel=(Δx,0,0) となり、エピポーラ線は正規化y座標一定の
# 水平線になる。導出（design §2.45の式に代入）: E=[t_rel]_x、sd=(x_b^T E x_a)^2/denom を
# 計算すると sd = delta^2 / 2（delta = ya-yb、正規化座標の差）となり、baseline(Δx)やx座標・
# ya自体には依存しない。よって d_px = sqrt(sd)*f_mean = |delta| / sqrt(2) * fx
# （f_mean=fx、両カメラ同一fxのため）。このヘルパ群はこの関係を使って対応点を合成する。
# ========================================

def _stereo_cam(x_offset: float, fx: float = 1000.0) -> dict:
    """R_rel=Iとなる横方向並進配置のカメラを作る（rvec=0固定、tvec=(x_offset,0,0)）。"""
    return _make_cam(fx, rvec=np.zeros(3), tvec=np.array([x_offset, 0.0, 0.0]))


def _stereo_matches(n: int, delta_norm, fx: float = 1000.0, cx: float = 960.0,
                    cy: float = 540.0, y0: float = 0.05, x_val: float = 0.0) -> tuple:
    """_stereo_cam同士の対応点uv座標を作る（正規化y座標差delta_normでSampson距離を制御）。

    delta_normはスカラーまたは(n,)配列。x_valは全点共通のx座標タグ（識別用、
    sd計算には影響しない）。
    """
    delta = np.broadcast_to(np.asarray(delta_norm, dtype=np.float64), (n,))
    ya = y0 + 0.001 * np.arange(n)
    yb = ya - delta
    xa = np.full(n, x_val)
    xb = np.full(n, x_val)
    u_a = np.stack([cx + fx * xa, cy + fx * ya], axis=1)
    u_b = np.stack([cx + fx * xb, cy + fx * yb], axis=1)
    return u_a, u_b


def _adopted_pair_matches(u_a: np.ndarray, u_b: np.ndarray) -> dict:
    return {"u_a": u_a, "u_b": u_b, "n": len(u_a), "status": "adopted"}


class TestEpiTolPxConstant:
    def test_epi_tol_px_is_3_0(self):
        assert EPI_TOL_PX == 3.0


class TestFilterCrossMatchesBasic:
    def test_epipolar_consistent_correspondences_survive(self):
        """design §2.45: エピポーラ整合対応（sd=0相当）は全点生存する"""
        cams = {"camA": _stereo_cam(0.0), "camB": _stereo_cam(1.0)}
        u_a, u_b = _stereo_matches(60, delta_norm=0.0)
        pairs = {("camA", "camB"): _adopted_pair_matches(u_a, u_b)}

        results = filter_cross_matches(cams, pairs)

        r = results[("camA", "camB")]
        assert r["n_before"] == 60
        assert r["n_survive"] == 60
        assert np.isclose(r["survival_rate"], 1.0)
        assert r["status"] == "adopted"

    def test_symmetric_false_match_outliers_removed(self):
        """design §2.45: 対称偽マッチ相当（エピポーラ幾何を大きく破る対応）は除去される"""
        cams = {"camA": _stereo_cam(0.0), "camB": _stereo_cam(1.0)}
        u_a_ok, u_b_ok = _stereo_matches(55, delta_norm=0.0)
        # 対称偽マッチ相当の外れ値: 正規化y座標を大きくずらしエピポーラ幾何を著しく破る
        u_a_bad, u_b_bad = _stereo_matches(5, delta_norm=0.5, y0=0.6)
        u_a = np.vstack([u_a_ok, u_a_bad])
        u_b = np.vstack([u_b_ok, u_b_bad])
        pairs = {("camA", "camB"): _adopted_pair_matches(u_a, u_b)}

        results = filter_cross_matches(cams, pairs)

        r = results[("camA", "camB")]
        assert r["n_before"] == 60
        assert r["n_survive"] == 55  # 外れ値5点のみ除去される
        assert r["status"] == "adopted"

    def test_boundary_epi_tol_3px_passes_4px_removed(self):
        """design §2.45 手順3: 換算誤差3px相当は通過、4px相当は除去される境界"""
        fx = 1000.0
        cams = {"camA": _stereo_cam(0.0, fx=fx), "camB": _stereo_cam(1.0, fx=fx)}

        delta_pass = 2.999 * np.sqrt(2) / fx  # d_px≒2.999px（EPI_TOL_PX=3.0以下→通過）
        delta_fail = 4.0 * np.sqrt(2) / fx    # d_px=4.0px（EPI_TOL_PX超過→除去）

        u_a_base, u_b_base = _stereo_matches(50, delta_norm=0.0)
        u_a_pass, u_b_pass = _stereo_matches(1, delta_norm=delta_pass, y0=0.9, x_val=0.11)
        u_a_fail, u_b_fail = _stereo_matches(1, delta_norm=delta_fail, y0=0.95, x_val=0.22)

        u_a = np.vstack([u_a_base, u_a_pass, u_a_fail])
        u_b = np.vstack([u_b_base, u_b_pass, u_b_fail])
        pairs = {("camA", "camB"): _adopted_pair_matches(u_a, u_b)}

        results = filter_cross_matches(cams, pairs)

        r = results[("camA", "camB")]
        assert r["n_before"] == 52
        assert r["n_survive"] == 51  # 3px相当点は生存、4px相当点は除去
        survived_xa = r["u_a"][:, 0]
        assert np.any(np.isclose(survived_xa, 960.0 + fx * 0.11))    # 3px相当点は残る
        assert not np.any(np.isclose(survived_xa, 960.0 + fx * 0.22))  # 4px相当点は消える


class TestFilterCrossMatchesDowngrade:
    def test_low_survival_downgrades_pair_without_breaking_connectivity(self):
        """design §2.45 手順4: 生存数 < N_PAIR_MIN のペアは不採用に降格する
        （連結性は他ペア経由で維持されるため全体はエラーにならない）"""
        cams = {"camA": _stereo_cam(0.0), "camB": _stereo_cam(1.0), "camC": _stereo_cam(2.0)}

        # A-B: 60点中20点だけ整合、40点は外れ値 → 生存20 < N_PAIR_MIN(50) で不採用に降格
        u_a_ok, u_b_ok = _stereo_matches(20, delta_norm=0.0)
        u_a_bad, u_b_bad = _stereo_matches(40, delta_norm=0.5, y0=0.6)
        u_a_ab = np.vstack([u_a_ok, u_a_bad])
        u_b_ab = np.vstack([u_b_ok, u_b_bad])

        # A-C, B-C: 全点整合（生存60 >= 50で採用のまま。連結性を維持する）
        u_a_ac, u_c_ac = _stereo_matches(60, delta_norm=0.0)
        u_b_bc, u_c_bc = _stereo_matches(60, delta_norm=0.0)

        pairs = {
            ("camA", "camB"): _adopted_pair_matches(u_a_ab, u_b_ab),
            ("camA", "camC"): _adopted_pair_matches(u_a_ac, u_c_ac),
            ("camB", "camC"): _adopted_pair_matches(u_b_bc, u_c_bc),
        }

        results = filter_cross_matches(cams, pairs)

        assert results[("camA", "camB")]["n_survive"] == 20
        assert results[("camA", "camB")]["status"] == "rejected"
        assert results[("camA", "camC")]["status"] == "adopted"
        assert results[("camB", "camC")]["status"] == "adopted"


class TestFilterCrossMatchesErrors:
    def test_disconnected_pose_confirmed_cameras_raise(self):
        """design §2.45 手順4: フィルタ後採用ペアグラフで全ポーズ確定カメラが単一連結成分に
        入らない場合はInputErrorを送出する（連結成分の内訳つき）"""
        cams = {
            "camA": _stereo_cam(0.0), "camB": _stereo_cam(1.0),
            "camC": _stereo_cam(2.0), "camD": _stereo_cam(3.0),
        }
        u_a_ab, u_b_ab = _stereo_matches(60, delta_norm=0.0)
        u_c_cd, u_d_cd = _stereo_matches(60, delta_norm=0.0)
        pairs = {
            ("camA", "camB"): _adopted_pair_matches(u_a_ab, u_b_ab),
            ("camC", "camD"): _adopted_pair_matches(u_c_cd, u_d_cd),
        }

        with pytest.raises(InputError):
            filter_cross_matches(cams, pairs)

    def test_all_pairs_rejected_raise(self):
        """design §2.45 手順4: 全ペア不採用の場合はInputErrorを送出する"""
        cams = {"camA": _stereo_cam(0.0), "camB": _stereo_cam(1.0)}
        u_a, u_b = _stereo_matches(60, delta_norm=0.5, y0=0.6)  # 全点がエピポーラ幾何を破る外れ値
        pairs = {("camA", "camB"): _adopted_pair_matches(u_a, u_b)}

        with pytest.raises(InputError):
            filter_cross_matches(cams, pairs)


# ========================================
# Stage A: アンカー対応点（design §2.4）。render_image / render_depth_alpha_distorted /
# call_matcher をmonkeypatchでモックし、collect_anchor_points の入出力契約
# （conf選別・ANCHOR_MAX_PTS切り詰め）のみを検証する（GPU・subprocess非依存）。
# ========================================

def _anchor_uniform_scene(shape=(20, 20), depth_value=2.0, alpha_value=1.0):
    """gate_breakdownの深度・α系条件が全点通過する一様シーンを作る。"""
    depth = np.full(shape, depth_value, dtype=np.float64)
    alpha = np.full(shape, alpha_value, dtype=np.float64)
    bgr = np.zeros((shape[0], shape[1], 3), dtype=np.uint8)
    return bgr, depth, alpha


class TestCollectAnchorPointsConfFilter:
    def test_conf_below_loftr_th_is_dropped(self, monkeypatch, tmp_path):
        """design §2.4 手順1: conf >= LOFTR_CONF_TH（=0.2）の選別を明示的に適用する"""
        cam = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        cam["name"] = "camA"
        bgr, depth, alpha = _anchor_uniform_scene()
        monkeypatch.setattr(aem, "render_image", lambda *a, **k: bgr)
        monkeypatch.setattr(aem, "render_depth_alpha_distorted",
                            lambda gaussians, cam, near_plane: (depth, alpha))

        # u_q == u_r（画素変位0）にして cond_disp を常に通過させ、conf選別のみを見る
        u = np.array([[5.0, 5.0], [8.0, 8.0], [10.0, 10.0], [12.0, 12.0]])
        conf = np.array([0.05, 0.15, 0.25, 0.35])
        monkeypatch.setattr(aem, "call_matcher",
                            lambda query_png, render_png, out_npz: (u.copy(), u.copy(),
                                                                     conf.copy(), {}))

        X_w, u_i = aem.collect_anchor_points(cam, gaussians={}, query_png=Path("dummy.png"),
                                             tmp_dir=tmp_path)

        assert len(X_w) == 2
        assert len(u_i) == 2
        survived = {tuple(p) for p in u_i}
        assert survived == {(10.0, 10.0), (12.0, 12.0)}  # conf>=0.2の2点のみ


class TestCollectAnchorPointsMaxPts:
    def test_truncates_to_anchor_max_pts_by_confidence_descending(self, monkeypatch, tmp_path):
        """design §2.4 手順3: ANCHOR_MAX_PTS超過時はLoFTR conf降順で切り詰める"""
        monkeypatch.setattr(aem, "ANCHOR_MAX_PTS", 2)
        cam = _make_cam(1000.0, rvec=np.zeros(3), tvec=np.zeros(3))
        cam["name"] = "camA"
        bgr, depth, alpha = _anchor_uniform_scene()
        monkeypatch.setattr(aem, "render_image", lambda *a, **k: bgr)
        monkeypatch.setattr(aem, "render_depth_alpha_distorted",
                            lambda gaussians, cam, near_plane: (depth, alpha))

        u = np.array([[5.0, 5.0], [8.0, 8.0], [10.0, 10.0], [12.0, 12.0]])
        conf = np.array([0.9, 0.3, 0.5, 0.7])  # 全点conf>=LOFTR_CONF_TH
        monkeypatch.setattr(aem, "call_matcher",
                            lambda query_png, render_png, out_npz: (u.copy(), u.copy(),
                                                                     conf.copy(), {}))

        X_w, u_i = aem.collect_anchor_points(cam, gaussians={}, query_png=Path("dummy.png"),
                                             tmp_dir=tmp_path)

        assert len(X_w) == 2  # ANCHOR_MAX_PTS=2に切り詰め
        assert len(u_i) == 2
        survived = {tuple(p) for p in u_i}
        assert survived == {(5.0, 5.0), (12.0, 12.0)}  # conf降順で上位2点（0.9, 0.7）


# ========================================
# main() の --init-cameras 全カメラ必須チェック（design §2.7）
# ========================================

class TestMainInitCamerasCheck:
    def test_target_camera_outside_init_cameras_raises_exit_1(self, tmp_path, capsys):
        """design §2.7: 対象カメラに--init-cameras外のカメラが含まれる場合はエラー終了1"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS + """
[camC]
name = "camC"
size = [1920.0, 1080.0]
matrix = [[1200.0, 0.0, 960.0], [0.0, 1200.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.03, 0.0, 0.0, 0.0]
rotation = [7.0, 7.0, 7.0]
translation = [7.0, 7.0, 7.0]
""")
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES_ALL)  # camA, camBのみ（camCは含まれない）

        ply_path = tmp_path / "dummy.ply"
        ply_path.write_text("dummy")
        images_dir = tmp_path / "images"
        images_dir.mkdir()

        argv = [
            "--toml", str(toml_path),
            "--ply", str(ply_path),
            "--images-dir", str(images_dir),
            "--init-toml", str(init_path),
            "--init-cameras", "camA", "camB",
            "--cameras", "camA", "camB", "camC",
            "--out-toml", str(tmp_path / "out.toml"),
            "--out-report", str(tmp_path / "out_report.txt"),
        ]

        ret = aem.main(argv)

        assert ret == 1
        captured = capsys.readouterr()
        assert "camC" in captured.err

    def test_pose_confirmed_cameras_below_2_raises_exit_1(self, tmp_path, capsys):
        """design §2.7/2.5: ポーズ確定カメラが2台未満の場合はStage Fに進まずエラー終了1"""
        toml_path = tmp_path / "intrinsics.toml"
        toml_path.write_text(_TOML_INTRINSICS)
        init_path = tmp_path / "init.toml"
        init_path.write_text(_TOML_INIT_POSES_ALL)

        ply_path = tmp_path / "dummy.ply"
        ply_path.write_text("dummy")
        images_dir = tmp_path / "images"
        images_dir.mkdir()

        argv = [
            "--toml", str(toml_path),
            "--ply", str(ply_path),
            "--images-dir", str(images_dir),
            "--init-toml", str(init_path),
            "--init-cameras", "camA",
            "--cameras", "camA",
            "--out-toml", str(tmp_path / "out.toml"),
            "--out-report", str(tmp_path / "out_report.txt"),
        ]

        ret = aem.main(argv)

        assert ret == 1
        captured = capsys.readouterr()
        assert "2台未満" in captured.err


# ========================================
# Stage D: ホールドアウト分割（design §2.6 前半、Step 5 スコープ）
# ========================================

class TestSplitHoldout:
    def test_reproducible_with_same_seed(self):
        """design §2.6: 同一シードで同一分割になる（再現性）"""
        u_a = np.arange(20, dtype=np.float64).reshape(10, 2)
        u_b = u_a + 100.0
        filter_results = {("camA", "camB"): {"status": "adopted", "u_a": u_a, "u_b": u_b}}

        r1 = split_holdout(filter_results, 0.2, seed=123)
        r2 = split_holdout(filter_results, 0.2, seed=123)

        key = ("camA", "camB")
        assert np.array_equal(r1[key]["train"]["u_a"], r2[key]["train"]["u_a"])
        assert np.array_equal(r1[key]["holdout"]["u_a"], r2[key]["holdout"]["u_a"])
        assert np.array_equal(r1[key]["train"]["u_b"], r2[key]["train"]["u_b"])
        assert np.array_equal(r1[key]["holdout"]["u_b"], r2[key]["holdout"]["u_b"])

    def test_different_seed_gives_different_split(self):
        u_a = np.arange(40, dtype=np.float64).reshape(20, 2)
        u_b = u_a.copy()
        filter_results = {("camA", "camB"): {"status": "adopted", "u_a": u_a, "u_b": u_b}}

        r1 = split_holdout(filter_results, 0.2, seed=1)
        r2 = split_holdout(filter_results, 0.2, seed=2)

        key = ("camA", "camB")
        assert not np.array_equal(r1[key]["holdout"]["u_a"], r2[key]["holdout"]["u_a"])

    def test_floor_ratio_behavior(self):
        """design §2.6: floor(N*ratio) 点がholdoutになる（N=10, ratio=0.25 → floor(2.5)=2）"""
        u_a = np.arange(20, dtype=np.float64).reshape(10, 2)
        u_b = u_a.copy()
        filter_results = {("camA", "camB"): {"status": "adopted", "u_a": u_a, "u_b": u_b}}

        r = split_holdout(filter_results, 0.25, seed=1)

        key = ("camA", "camB")
        assert len(r[key]["holdout"]["u_a"]) == 2
        assert len(r[key]["train"]["u_a"]) == 8

    def test_non_adopted_pairs_excluded(self):
        """design §2.6: 対象は status=='adopted' のペアのみ"""
        filter_results = {
            ("camA", "camB"): {"status": "rejected",
                               "u_a": np.zeros((5, 2)), "u_b": np.zeros((5, 2))},
        }
        r = split_holdout(filter_results, 0.2, seed=1)
        assert r == {}


# ========================================
# Stage D: 三角測量の座標方式（design §2.5「クロス3D点の初期化」）
# ========================================

class TestTriangulateNormalized:
    def test_round_trip_recovers_known_point_from_known_poses(self):
        """design §2.5: 正規化座標 + [R|t] 方式。既知の合成ポーズ・3D点で往復一致する"""
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(4)
        rvec_a = np.array([0.05, -0.02, 0.01])
        tvec_a = np.array([0.1, 0.0, 2.0])
        rvec_b = np.array([-0.03, 0.10, -0.02])
        tvec_b = np.array([-0.5, 0.05, 2.2])
        X_true = np.array([0.3, -0.1, 3.0])

        def project(rvec, tvec):
            proj, _ = cv2.projectPoints(X_true.reshape(1, 1, 3), rvec.reshape(3, 1),
                                        tvec.reshape(3, 1), K, D)
            return proj.reshape(1, 2)

        u_a = project(rvec_a, tvec_a)
        u_b = project(rvec_b, tvec_b)
        cam_a = {"K": K, "D": D, "rvec": rvec_a, "tvec": tvec_a}
        cam_b = {"K": K, "D": D, "rvec": rvec_b, "tvec": tvec_b}

        Y, angle_deg, depth_a, depth_b = aem._triangulate_normalized(u_a, u_b, cam_a, cam_b)

        assert np.allclose(Y[0], X_true, atol=1e-4)
        assert depth_a[0] > 0.0
        assert depth_b[0] > 0.0
        assert angle_deg[0] > 0.0


# ========================================
# Stage D: 交会角重み（design §2.5、VISAPP2026準拠のガウス型スコア）
# ========================================

class TestCrossingAngleWeight:
    def test_weight_1_at_optimal_angle(self):
        w = crossing_angle_weight(np.array([aem.THETA_OPT_DEG]))
        assert np.isclose(w[0], 1.0)

    def test_monotonic_decay_away_from_optimal(self):
        """design §2.5: θ=TRI_MIN_ANGLE_DEGとθ=90°で単調減衰
        （最適角30°から遠いほど重みが小さくなる）"""
        w_opt = crossing_angle_weight(np.array([aem.THETA_OPT_DEG]))[0]
        w_min = crossing_angle_weight(np.array([TRI_MIN_ANGLE_DEG]))[0]
        w_90 = crossing_angle_weight(np.array([90.0]))[0]

        assert np.isclose(w_opt, 1.0)
        assert w_min < w_opt
        assert w_90 < w_opt
        assert w_90 < w_min  # 90度の方が2度より最適角(30度)から遠いため重みが小さい


# ========================================
# Stage D: 残差NaN置換バリア（design §2.5 エラーハンドリング）
# ========================================

class TestProjectedResidualNaNBarrier:
    def test_nan_point_replaced_with_1e3px(self):
        """3D点がNaN（背面に回った場合等に発生し得る）だと投影結果がNaNになり、
        1e3pxの定数バリアに置換される"""
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(4)
        points = np.array([[np.nan, np.nan, np.nan]])
        uv = np.array([[100.0, 100.0]])

        resid = aem._projected_residual(points, np.zeros(3), np.zeros(3), K, D, uv, weight=1.0)

        assert resid.shape == (2,)
        assert np.allclose(resid, [1e3, 1e3])

    def test_finite_point_not_affected_by_barrier(self):
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(4)
        points = np.array([[0.0, 0.0, 1.0]])  # rvec=0,tvec=0で(cx,cy)に投影される
        uv = np.array([[960.0, 540.0]])

        resid = aem._projected_residual(points, np.zeros(3), np.zeros(3), K, D, uv, weight=1.0)

        assert np.allclose(resid, [0.0, 0.0])


# ========================================
# Stage D: BAProblem の jac_sparsity（design §1.7/§2.5）
# ========================================

def _simple_cam(fx: float, tvec) -> dict:
    K = np.array([[fx, 0.0, 960.0], [0.0, fx, 540.0], [0.0, 0.0, 1.0]])
    return {"K": K, "D": np.zeros(4), "rvec": np.zeros(3), "tvec": np.asarray(tvec, dtype=np.float64)}


class TestBAProblemSparsity:
    def test_shape_and_nonzero_block_positions(self):
        """design §2.5: 残差数×未知数の疎行列で、各残差ブロックは対応カメラ/3D点の
        パラメータ列のみ非ゼロになる"""
        cams = {"camA": _simple_cam(1000.0, [0.0, 0.0, 0.0]),
               "camB": _simple_cam(1000.0, [1.0, 0.0, 0.0])}
        anchor = {
            "camA": (np.zeros((3, 3)), np.zeros((3, 2))),
            "camB": (np.zeros((3, 3)), np.zeros((3, 2))),
        }
        cross_train = [{"cam_a": "camA", "cam_b": "camB",
                       "u_a": np.zeros(2), "u_b": np.zeros(2),
                       "Y0": np.zeros(3), "weight": 1.0}]

        problem = build_ba_problem(cams, anchor, cross_train)

        # 未知数: camA(6) + camB(6) + 点1個(3) = 15。残差: アンカー(2*3+2*3=12) + クロス(4*1=4) = 16
        assert problem.x0.shape == (15,)
        assert problem.sparsity.shape == (16, 15)

        # camAアンカー行(0:6)はcamA列(0:6)のみ非ゼロ
        assert problem.sparsity[0:6, 0:6].nnz == 36
        assert problem.sparsity[0:6, 6:12].nnz == 0
        assert problem.sparsity[0:6, 12:15].nnz == 0

        # camBアンカー行(6:12)はcamB列(6:12)のみ非ゼロ
        assert problem.sparsity[6:12, 6:12].nnz == 36
        assert problem.sparsity[6:12, 0:6].nnz == 0
        assert problem.sparsity[6:12, 12:15].nnz == 0

        # クロス行前半(12:14, camA投影)はcamA列と点列のみ非ゼロ
        assert problem.sparsity[12:14, 0:6].nnz == 12
        assert problem.sparsity[12:14, 12:15].nnz == 6
        assert problem.sparsity[12:14, 6:12].nnz == 0

        # クロス行後半(14:16, camB投影)はcamB列と点列のみ非ゼロ
        assert problem.sparsity[14:16, 6:12].nnz == 12
        assert problem.sparsity[14:16, 12:15].nnz == 6
        assert problem.sparsity[14:16, 0:6].nnz == 0

    def test_unpack_recovers_x0_layout(self):
        cams = {"camA": _simple_cam(1000.0, [0.1, 0.2, 3.0])}
        anchor = {"camA": (np.zeros((3, 3)), np.zeros((3, 2)))}
        cross_train = []

        problem = build_ba_problem(cams, anchor, cross_train)
        poses, Y = problem.unpack(problem.x0)

        assert np.allclose(poses["camA"][0], [0.0, 0.0, 0.0])
        assert np.allclose(poses["camA"][1], [0.1, 0.2, 3.0])
        assert Y.shape == (0, 3)


# ========================================
# Stage D: 非悪化フォールバック（design §2.5、FR-005）
# ========================================

class TestApplyBaFallback:
    def _build_case(self, dx: float):
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(4)
        cams_before = {"camA": {"K": K, "D": D, "rvec": np.zeros(3), "tvec": np.zeros(3)}}
        X_w = np.array([[0.0, 0.0, 1.0]])
        u_i = np.array([[960.0, 540.0]])  # BA前ポーズで完全一致投影（残差中央値0）
        anchor = {"camA": (X_w, u_i)}
        rvec_after = np.zeros(3)
        tvec_after = np.array([dx, 0.0, 0.0])  # x方向にdxずらして残差をdx*fx[px]発生させる
        ba_result = BAResult(poses={"camA": (rvec_after, tvec_after)},
                             status=1, cost=0.0, cost_initial=0.0, nfev=1)
        return cams_before, anchor, ba_result

    def test_degrade_above_tolerance_reverts_to_before_pose(self):
        """悪化 0.31px（>BA_DEGRADE_TOL_PX=0.3px）→ BA前ポーズに復帰"""
        cams_before, anchor, ba_result = self._build_case(dx=0.00031)

        final_poses, records = apply_ba_fallback(cams_before, ba_result, anchor)

        rec = records[0]
        assert np.isclose(rec["degrade_px"], 0.31, atol=1e-3)
        assert rec["fallback"] is True
        assert np.allclose(final_poses["camA"][1], cams_before["camA"]["tvec"])

    def test_degrade_below_tolerance_keeps_ba_pose(self):
        """悪化 0.29px（<BA_DEGRADE_TOL_PX=0.3px）→ BA後ポーズを維持"""
        cams_before, anchor, ba_result = self._build_case(dx=0.00029)

        final_poses, records = apply_ba_fallback(cams_before, ba_result, anchor)

        rec = records[0]
        assert np.isclose(rec["degrade_px"], 0.29, atol=1e-3)
        assert rec["fallback"] is False
        assert np.allclose(final_poses["camA"][1], ba_result.poses["camA"][1])


# ========================================
# Stage D: BA参加条件の境界（design §2.4 手順4、FR-005、N_ANCHOR_MIN=30）
# ========================================

class TestSelectBaCameraNames:
    def test_n_a_29_excluded(self):
        """N_A=29はBA除外（信頼初期ポーズ維持）"""
        anchor_results = {"camA": (np.zeros((29, 3)), np.zeros((29, 2)))}
        names = select_ba_camera_names(["camA"], anchor_results)
        assert names == []

    def test_n_a_30_participates(self):
        """N_A=30はBA参加"""
        anchor_results = {"camA": (np.zeros((30, 3)), np.zeros((30, 2)))}
        names = select_ba_camera_names(["camA"], anchor_results)
        assert names == ["camA"]

    def test_n_anchor_min_is_30(self):
        assert N_ANCHOR_MIN == 30


# ========================================
# roadmap Step 5: 合成データによるBA復元検証
# （既知の8ポーズ+3D点から生成した観測に摂動を与え、BAが復元することを確認する）
# ========================================

def _look_at_pose(angle: float, radius: float = 5.0, height: float = 0.0,
                  target: np.ndarray = np.array([0.0, 0.0, 0.5])) -> tuple[np.ndarray, np.ndarray]:
    C = np.array([radius * np.cos(angle), radius * np.sin(angle), height])
    z_axis = target - C
    z_axis /= np.linalg.norm(z_axis)
    world_up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(z_axis, world_up)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    R = np.stack([x_axis, y_axis, z_axis], axis=0)
    t = -R @ C
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten(), t


class TestSyntheticBundleAdjustmentRecovery:
    def test_ba_recovers_perturbed_8_camera_rig(self):
        rng = np.random.default_rng(42)
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(4)
        n_cams = 8
        names = [f"cam{i}" for i in range(n_cams)]

        true_pose = {}
        for i, name in enumerate(names):
            angle = 2 * np.pi * i / n_cams
            true_pose[name] = _look_at_pose(angle)

        # BA前（信頼初期）ポーズ = 真値に小さな摂動を加えたもの
        init_cams = {}
        for name in names:
            rvec_t, tvec_t = true_pose[name]
            d_rvec = rng.normal(scale=0.02, size=3)
            d_tvec = rng.normal(scale=0.03, size=3)
            init_cams[name] = {"K": K, "D": D,
                               "rvec": rvec_t + d_rvec, "tvec": tvec_t + d_tvec}

        # アンカー対応点: 各カメラ40点。真ポーズでの投影を観測u_iとする
        anchor = {}
        n_anchor = 40
        for name in names:
            rvec_t, tvec_t = true_pose[name]
            X_w = rng.uniform(-0.5, 0.5, size=(n_anchor, 3)) + np.array([0.0, 0.0, 0.5])
            proj, _ = cv2.projectPoints(X_w.reshape(-1, 1, 3), rvec_t.reshape(3, 1),
                                        tvec_t.reshape(3, 1), K, D)
            anchor[name] = (X_w, proj.reshape(-1, 2))

        # クロス対応点: 隣接ペアごとに15点。真ポーズでの投影を観測とする
        cross_pairs = {}
        m_per_pair = 15
        for i in range(n_cams):
            a, b = names[i], names[(i + 1) % n_cams]
            key = tuple(sorted([a, b]))
            X_w = rng.uniform(-0.4, 0.4, size=(m_per_pair, 3)) + np.array([0.0, 0.0, 0.5])
            rvec_a, tvec_a = true_pose[a]
            rvec_b, tvec_b = true_pose[b]
            proj_a, _ = cv2.projectPoints(X_w.reshape(-1, 1, 3), rvec_a.reshape(3, 1),
                                          tvec_a.reshape(3, 1), K, D)
            proj_b, _ = cv2.projectPoints(X_w.reshape(-1, 1, 3), rvec_b.reshape(3, 1),
                                          tvec_b.reshape(3, 1), K, D)
            u_a, u_b = proj_a.reshape(-1, 2), proj_b.reshape(-1, 2)
            if a != key[0]:
                u_a, u_b = u_b, u_a
            cross_pairs[key] = {"u_a": u_a, "u_b": u_b}

        cross_train = build_cross_points(init_cams, cross_pairs)
        assert len(cross_train) > 0

        problem = build_ba_problem(init_cams, anchor, cross_train)
        result = run_bundle_adjustment(problem)

        assert result.status >= 1
        assert result.cost <= result.cost_initial

        for name in names:
            rvec_t, tvec_t = true_pose[name]
            rvec_r, tvec_r = result.poses[name]
            R_t, _ = cv2.Rodrigues(rvec_t.reshape(3, 1))
            R_r, _ = cv2.Rodrigues(np.asarray(rvec_r, dtype=np.float64).reshape(3, 1))
            dR = R_t.T @ R_r
            ang_err_deg = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)))
            trans_err_m = float(np.linalg.norm(tvec_t - np.asarray(tvec_r)))
            assert ang_err_deg < 0.05, f"{name}: rot_err={ang_err_deg}deg"
            assert trans_err_m < 0.01, f"{name}: trans_err={trans_err_m}m"


# ========================================
# write_multiview_toml（design §2.6「TOML」、§4「出力TOMLの往復」「poses限定」）
# ========================================

_INPUT_TOML_MULTIVIEW = """\
[camA]
name = "camA"
size = [1920.0, 1080.0]
matrix = [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.04, 0.01, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 3.0]
fisheye = false

[camB]
name = "camB"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.02, 0.001, -0.002]
rotation = [0.1, 0.2, 0.3]
translation = [0.5, -0.3, 5.0]
fisheye = false

[camC]
name = "camC"
size = [1920.0, 1080.0]
matrix = [[1050.0, 0.0, 960.0], [0.0, 1050.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.03, 0.0, 0.0, 0.0]
rotation = [0.4, 0.5, 0.6]
translation = [1.0, 1.0, 4.0]
fisheye = false
"""


class TestWriteMultiviewToml:
    def _write_input_toml(self, tmp_path) -> Path:
        p = tmp_path / "intrinsics_all.toml"
        p.write_text(_INPUT_TOML_MULTIVIEW)
        return p

    def test_round_trip_matches_load_cameras_toml(self, tmp_path):
        """write_multiview_toml -> load_cameras_toml で読み戻した値が一致する
        （design §4「出力TOMLの往復」）。"""
        input_toml = self._write_input_toml(tmp_path)
        out_toml = tmp_path / "out.toml"
        rvec_a = np.array([9.0, 8.0, 7.0])
        tvec_a = np.array([1.0, 2.0, 3.0])
        rvec_b = np.array([-1.0, -2.0, -3.0])
        tvec_b = np.array([0.1, 0.2, 0.3])
        poses = {"camA": (rvec_a, tvec_a), "camB": (rvec_b, tvec_b)}

        write_multiview_toml(str(input_toml), str(out_toml), poses)

        cams = load_cameras_toml(str(out_toml))
        assert np.allclose(cams["camA"]["rvec"], rvec_a)
        assert np.allclose(cams["camA"]["tvec"], tvec_a)
        assert np.allclose(cams["camA"]["K"],
                           [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]])
        assert np.allclose(cams["camA"]["D"], [-0.04, 0.01, 0.0, 0.0])
        assert cams["camA"]["width"] == 1920
        assert cams["camA"]["height"] == 1080

        assert np.allclose(cams["camB"]["rvec"], rvec_b)
        assert np.allclose(cams["camB"]["tvec"], tvec_b)
        assert np.allclose(cams["camB"]["K"],
                           [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        assert np.allclose(cams["camB"]["D"], [-0.05, 0.02, 0.001, -0.002])

    def test_only_cameras_in_poses_are_written(self, tmp_path):
        """poses に含まれないカメラのセクションは出力TOMLに存在しない
        （design §4「poses限定」、既存write_output_tomlとの差異）。"""
        input_toml = self._write_input_toml(tmp_path)
        out_toml = tmp_path / "out.toml"
        poses = {"camB": (np.array([0.1, 0.2, 0.3]), np.array([0.5, -0.3, 5.0]))}

        write_multiview_toml(str(input_toml), str(out_toml), poses)

        with open(out_toml, "rb") as f:
            data = tomli.load(f)

        assert list(data.keys()) == ["camB"]
        assert "camA" not in data
        assert "camC" not in data


# ========================================
# GPU・subprocess非依存であること（torchをimportしないこと）
# ========================================

class TestTorchIndependence:
    def test_import_does_not_pull_in_torch(self):
        """adjust_extrinsics_multiview のimportだけではtorchがsys.modulesに入らない
        （render_depth_alpha_distorted等のGPU依存処理は関数内で遅延importされるため）"""
        code = (
            "import sys\n"
            "sys.path.insert(0, 'phase4')\n"
            "import adjust_extrinsics_multiview\n"
            "assert 'torch' not in sys.modules, sorted(sys.modules)\n"
            "print('OK')\n"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=str(Path(__file__).parent.parent),
            capture_output=True, text=True)
        assert result.returncode == 0, f"stdout={result.stdout}\nstderr={result.stderr}"
        assert "OK" in result.stdout
