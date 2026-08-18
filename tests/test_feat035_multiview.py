"""feat-035: phase4/adjust_extrinsics_multiview.py の単体テスト（Step 2〜3: 本体骨格 + Stage P/B）

design.md §4「テスト設計」のうち Step 2〜3 範囲（ペア列挙・採用判定・カメラ辞書マージ規則・
gate_depth_quality・ブートストラップの貪欲順序と確定fail条件）を検証する。subprocess・GPUに
依存しない純粋関数のみを対象とする。run_pair_matching 自体（mast3r_cli.py の subprocess
呼び出し部）と、実際のGPUレンダを要する render_depth_alpha_distorted の中身は本テスト範囲外
とする（design §4 注記どおり、純粋関数部分を分離してテストする方針。bootstrap_cameras の
テストでは render_depth_alpha_distorted と cv2.solvePnPRansac を monkeypatch でモックする）。
"""
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import adjust_extrinsics_multiview as aem
from adjust_extrinsics_multiview import (
    N_PAIR_MIN,
    bootstrap_cameras,
    build_camera_dict,
    enumerate_pairs,
    gate_depth_quality,
    is_pair_adopted,
)


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
