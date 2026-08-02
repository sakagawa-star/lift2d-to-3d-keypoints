"""feat-026: phase4/refine_extrinsics.py の単体テスト

design.md §8 に定義された5系統（実データ・GPUに依存しない部分）:
  1. クラスタ分析（§4.5）: 合成サンプル（単峰60・二峰42+18・多峰）でf_c/f2/受理分類
  2. 逆写像（§4.9の式）: 格子点round-trip、既知座標の順逆一致
     （loftr_cli.pyはmatcher_lab環境専用でtorchをtop-level importするため、
       root環境では import できない。design §3.1「座標逆写像」の式そのものを
       ここで直接検証する）
  3. ゲート逐次分類（§4.3-3）: 人工の深度・αマップで除外内訳の合計が一致
  4. TOML出力（§4.7）: 入力→出力でK・歪みが不変、rotation/translationのみ置換、
     load_cameras_tomlで再読込可能
  5. pose_diff・摂動生成: 既知入力での数値一致
  6. 入力検証の点数条件・手動点PnP・仲裁の多点ケース（§4.1/§4.2/§4.5、ステップ7
     差し戻しイテレーション1: 一意ObjectName「ちょうど6個」→「6個以上・全点使用」）
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import tomli

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from refine_extrinsics import (
    F2_BIMODAL,
    F_C_MIN,
    ORIG_H,
    ORIG_W,
    _format_camera_section,
    _format_toml_value,
    cluster_analysis,
    decide_acceptance,
    gate_breakdown,
    median_reproj_px,
    perturb_pose,
    pnp_manual,
    pose_diff,
    validate_camera,
    write_output_toml,
)
from render_keypoints import load_cameras_toml


def _make_pose(center, axis_deg=(0.0, 0.0, 0.0)):
    """カメラ中心[m]と回転角[deg](XYZ軸別)から (rvec, tvec) を作る"""
    rx, ry, rz = np.deg2rad(axis_deg)
    R, _ = cv2.Rodrigues(np.array([rx, ry, rz], dtype=np.float64))
    rvec, _ = cv2.Rodrigues(R)
    tvec = -R @ np.asarray(center, dtype=np.float64)
    return rvec.flatten(), tvec.flatten()


# ========================================
# 1. クラスタ分析（design §4.5, §8-1）
# ========================================

class TestClusterAnalysis:
    def test_unimodal_60_accepted(self):
        """単峰60サンプル(微小ノイズのみ)はf_c高い・第2クラスタ低い(受理条件を満たす)"""
        rng = np.random.default_rng(0)
        samples = []
        for _ in range(60):
            c = np.array([0.0, 0.0, 5.0]) + rng.normal(scale=0.001, size=3)
            samples.append(_make_pose(c, axis_deg=rng.normal(scale=0.01, size=3)))
        cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(samples)
        assert len(cluster) == 60
        assert f_c >= F_C_MIN
        assert f2 < F2_BIMODAL

    def test_bimodal_42_18(self):
        """二峰42+18サンプル: 第1クラスタ占有率42/60、第2クラスタ占有率18/60"""
        rng = np.random.default_rng(1)
        samples = []
        center1 = np.array([0.0, 0.0, 5.0])
        center2 = np.array([0.5, 0.0, 5.0])  # R_POS(2cm)より十分離す
        for _ in range(42):
            samples.append(_make_pose(center1 + rng.normal(scale=0.001, size=3)))
        for _ in range(18):
            samples.append(_make_pose(center2 + rng.normal(scale=0.001, size=3)))
        cluster, f_c, f2, _, _, _ = cluster_analysis(samples)
        assert len(cluster) == 42
        assert abs(f_c - 42 / 60) < 1e-9
        assert abs(f2 - 18 / 60) < 1e-9
        assert f2 >= F2_BIMODAL

    def test_multimodal_scattered_low_agreement(self):
        """多峰(互いに遠く離れた60点): 最大クラスタ占有率が受理閾値未満"""
        samples = []
        for i in range(60):
            c = np.array([i * 0.5, 0.0, 5.0])
            samples.append(_make_pose(c))
        cluster, f_c, f2, _, _, _ = cluster_analysis(samples)
        assert f_c < F_C_MIN
        assert len(cluster) < 60


# ========================================
# 2. 逆写像（design §3.1「座標逆写像」の式、§8-2）
# ========================================

class TestInverseMapping:
    """design §3.1: orig=(small+0.5)/scale-0.5、scale=1280/1920=2/3 の式を直接検証する。

    loftr_cli.pyはmatcher_lab環境専用（torchをtop-level import）でrootのpytest環境
    からはimportできないため、式そのものをここで再現して検証する。
    """

    SCALE = 1280 / 1920

    @staticmethod
    def _fwd(u, scale):
        return (u + 0.5) * scale - 0.5

    @staticmethod
    def _inv(u, scale):
        return (u + 0.5) / scale - 0.5

    def test_grid_roundtrip_within_tolerance(self):
        gx = np.linspace(0, 1920 - 1, 5)
        gy = np.linspace(0, 1080 - 1, 5)
        grid = np.array([[x, y] for x in gx for y in gy], dtype=np.float64)
        rt = np.abs(self._inv(self._fwd(grid, self.SCALE), self.SCALE) - grid).max()
        assert rt <= 0.5

    def test_known_coordinate_forward_inverse_consistency(self):
        u_small = np.array([[0.0, 0.0], [639.5, 359.5], [1279.0, 719.0]])
        u_orig = self._inv(u_small, self.SCALE)
        u_small_back = self._fwd(u_orig, self.SCALE)
        assert np.allclose(u_small, u_small_back, atol=1e-9)

    def test_known_orig_to_small_and_back(self):
        u_orig = np.array([[0.0, 0.0], [960.0, 540.0], [1919.0, 1079.0]])
        u_small = self._fwd(u_orig, self.SCALE)
        u_orig_back = self._inv(u_small, self.SCALE)
        assert np.allclose(u_orig, u_orig_back, atol=1e-9)


# ========================================
# 3. ゲート逐次分類（design §4.3-3, §8-3）
# ========================================

class TestGateBreakdown:
    def test_exclusion_breakdown_targeted_buckets(self):
        """5点をそれぞれ異なる段で除外させ、逐次分類(depth_ok→α→z→分散)が
        正しいバケットに割り当てることを検証する"""
        H, W = 50, 20
        depth = np.full((H, W), 3.0, dtype=np.float32)
        alpha = np.full((H, W), 0.9, dtype=np.float32)
        var_map = np.zeros((H, W), dtype=np.float64)

        # band1 (行10-19): 深度NaN → excl_depth_nan
        depth[10:20, :] = np.nan
        # band2 (行20-29): 低α → excl_alpha
        alpha[20:30, :] = 0.1
        # band3 (行30-39): 深度範囲外(>10.0) → excl_z
        depth[30:40, :] = 20.0
        # band4 (行40-49): 高分散 → excl_var
        var_map[40:50, :] = 100.0

        u_r = np.array([[5.0, 5.0], [5.0, 15.0], [5.0, 25.0], [5.0, 35.0], [5.0, 45.0]])
        u_q = u_r.copy()  # 変位ゼロ(全点が変位ゲートを通過)
        tau_d = 1.0  # tau_d^2 = 1.0（band4のvar=100は超過、他は0で通過）
        tau_px = 5.0

        bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d, tau_px)

        assert bd["n_disp"] == 5
        assert bd["excl_depth_nan"] == 1
        assert bd["excl_alpha"] == 1
        assert bd["excl_z"] == 1
        assert bd["excl_var"] == 1
        assert bd["N"] == 1
        assert bd["keep"].tolist() == [True, False, False, False, False]
        # 除外内訳の合計 + 保持数 = 変位ゲート通過数（design §4.3 accept基準6の逐次分類）
        excl_sum = (bd["excl_depth_nan"] + bd["excl_alpha"]
                    + bd["excl_z"] + bd["excl_var"])
        assert bd["N"] + excl_sum == bd["n_disp"]

    def test_displacement_gate_excludes_before_counting(self):
        """変位ゲートを通らない点はn_dispにもexcl内訳にも計上されない"""
        H, W = 10, 10
        depth = np.full((H, W), 3.0, dtype=np.float32)
        alpha = np.full((H, W), 0.9, dtype=np.float32)
        var_map = np.zeros((H, W), dtype=np.float64)
        u_r = np.array([[5.0, 5.0], [5.0, 5.0]])
        u_q = np.array([[5.0, 5.0], [500.0, 500.0]])  # 2点目は大変位
        bd = gate_breakdown(u_q, u_r, depth, alpha, var_map, tau_d=1.0, tau_px=5.0)
        assert bd["n_disp"] == 1
        assert bd["N"] == 1


# ========================================
# 4. TOML出力（design §4.7, §8-4）
# ========================================

_INPUT_TOML = """\
[metadata]
source = "test"

[camB]
name = "camB"
size = [1920.0, 1080.0]
matrix = [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.05, 0.02, 0.001, -0.002]
rotation = [0.1, 0.2, 0.3]
translation = [0.5, -0.3, 5.0]
fisheye = false

[camA]
name = "camA"
size = [1920.0, 1080.0]
matrix = [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]]
distortions = [-0.04, 0.01, 0.0, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 3.0]
fisheye = false
"""


class TestWriteOutputToml:
    def _write_input_toml(self, tmp_path) -> Path:
        p = tmp_path / "input.toml"
        p.write_text(_INPUT_TOML)
        return p

    def test_accepted_camera_replaced_others_kept_order_preserved(self, tmp_path):
        input_toml = self._write_input_toml(tmp_path)
        out_toml = tmp_path / "output.toml"
        new_rvec = np.array([9.0, 8.0, 7.0])
        new_tvec = np.array([1.0, 2.0, 3.0])
        results = {"camB": {"accepted": True, "rvec_final": new_rvec, "tvec_final": new_tvec}}

        write_output_toml(str(input_toml), str(out_toml), results)

        with open(out_toml, "rb") as f:
            data = tomli.load(f)

        # 入力TOMLの全カメラを元の順序で含む
        assert list(data.keys()) == ["metadata", "camB", "camA"]

        # camB(受理): rotation/translationのみ置換、K・歪み・fisheyeは不変
        assert np.allclose(data["camB"]["rotation"], new_rvec)
        assert np.allclose(data["camB"]["translation"], new_tvec)
        assert np.allclose(data["camB"]["matrix"],
                           [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        assert np.allclose(data["camB"]["distortions"], [-0.05, 0.02, 0.001, -0.002])
        assert data["camB"]["fisheye"] is False

        # camA(結果に含まれない=非対象/不受理相当): 全キー入力値のまま
        assert np.allclose(data["camA"]["rotation"], [0.0, 0.0, 0.0])
        assert np.allclose(data["camA"]["translation"], [0.0, 0.0, 3.0])
        assert np.allclose(data["camA"]["matrix"],
                           [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]])

        # metadata: 転記 + refined_by/refined_dateを追記
        assert data["metadata"]["source"] == "test"
        assert data["metadata"]["refined_by"] == "feat-026"
        assert "refined_date" in data["metadata"]

    def test_rejected_camera_not_in_results_kept_unchanged(self, tmp_path):
        input_toml = self._write_input_toml(tmp_path)
        out_toml = tmp_path / "output.toml"
        # camBはresultsにあるがaccepted=False（不受理・失敗相当）→ 入力値のまま
        results = {"camB": {"accepted": False}}
        write_output_toml(str(input_toml), str(out_toml), results)
        with open(out_toml, "rb") as f:
            data = tomli.load(f)
        assert np.allclose(data["camB"]["rotation"], [0.1, 0.2, 0.3])
        assert np.allclose(data["camB"]["translation"], [0.5, -0.3, 5.0])

    def test_output_reloadable_by_load_cameras_toml(self, tmp_path):
        input_toml = self._write_input_toml(tmp_path)
        out_toml = tmp_path / "output.toml"
        write_output_toml(str(input_toml), str(out_toml), {})

        cams = load_cameras_toml(str(out_toml))
        assert set(cams.keys()) == {"camB", "camA"}
        assert cams["camA"]["width"] == 1920
        assert cams["camA"]["height"] == 1080
        assert np.allclose(cams["camA"]["K"],
                           [[1100.0, 0.0, 960.0], [0.0, 1100.0, 540.0], [0.0, 0.0, 1.0]])

    def test_nested_table_value_raises_value_error(self, tmp_path):
        """転記方法が定義できない型(テーブル内テーブル)はValueErrorで安全側に倒す"""
        bad = _INPUT_TOML + "\n[camA.sub]\nx = 1\n"
        p = tmp_path / "bad.toml"
        p.write_text(bad)
        out_toml = tmp_path / "output.toml"
        with pytest.raises(ValueError):
            write_output_toml(str(p), str(out_toml), {})


class TestFormatTomlValue:
    def test_bool(self):
        assert _format_toml_value(True) == "true"
        assert _format_toml_value(False) == "false"

    def test_str_is_quoted(self):
        assert _format_toml_value("camA") == '"camA"'

    def test_nested_list(self):
        assert _format_toml_value([[1.0, 0.0], [0.0, 1.0]]) == "[[1.0, 0.0], [0.0, 1.0]]"

    def test_dict_raises(self):
        with pytest.raises(ValueError):
            _format_toml_value({"x": 1})

    def test_camera_section_overrides_only_given_keys(self):
        cam_data = {"name": "camA", "rotation": [0.0, 0.0, 0.0], "translation": [0.0, 0.0, 0.0]}
        text = _format_camera_section("camA", cam_data, rvec=[1.0, 2.0, 3.0], tvec=None)
        assert "rotation = [1.0, 2.0, 3.0]" in text
        assert "translation = [0.0, 0.0, 0.0]" in text


# ========================================
# 5. pose_diff・摂動生成（design §4.6/§3.1, §8-5）
# ========================================

class TestPoseDiffAndPerturb:
    def test_pose_diff_identity_is_zero(self):
        rvec = np.array([0.1, -0.2, 0.3])
        tvec = np.array([0.5, -0.3, 5.0])
        dpos, dang = pose_diff(rvec, tvec, rvec, tvec)
        assert dpos == pytest.approx(0.0, abs=1e-9)
        # Rodrigues往復の浮動小数点誤差(1e-6 deg オーダー)を許容
        assert dang == pytest.approx(0.0, abs=1e-4)

    def test_pose_diff_known_translation_only(self):
        rvec = np.zeros(3)  # R=I → カメラ中心 C=-t
        dpos, dang = pose_diff(rvec, np.array([0.0, 0.0, 5.0]),
                               rvec, np.array([0.0, 0.0, 6.0]))
        assert dpos == pytest.approx(1.0, abs=1e-9)
        assert dang == pytest.approx(0.0, abs=1e-9)

    def test_pose_diff_known_rotation_only(self):
        rvec_a = np.zeros(3)
        rvec_b = np.array([0.0, np.deg2rad(10.0), 0.0])  # Y軸周り10度
        tvec = np.array([0.0, 0.0, 5.0])
        _, dang = pose_diff(rvec_a, tvec, rvec_b, tvec)
        assert dang == pytest.approx(10.0, abs=1e-6)

    def test_perturb_pose_known_magnitude(self):
        rvec0 = np.zeros(3)
        tvec0 = np.array([0.0, 0.0, 5.0])
        rng = np.random.default_rng(11)
        rvec1, tvec1 = perturb_pose(rvec0, tvec0, 0.05, 2.0, rng)
        dpos, dang = pose_diff(rvec1, tvec1, rvec0, tvec0)
        assert dpos == pytest.approx(0.05, abs=1e-9)
        assert dang == pytest.approx(2.0, abs=1e-6)

    def test_perturb_pose_deterministic_with_same_seed(self):
        rvec0 = np.array([0.1, 0.2, 0.3])
        tvec0 = np.array([0.5, -0.3, 5.0])
        rvec_a, tvec_a = perturb_pose(rvec0, tvec0, 0.05, 2.0, np.random.default_rng(11))
        rvec_b, tvec_b = perturb_pose(rvec0, tvec0, 0.05, 2.0, np.random.default_rng(11))
        assert np.allclose(rvec_a, rvec_b)
        assert np.allclose(tvec_a, tvec_b)


# ========================================
# 6. 入力検証の点数条件・手動点PnP・仲裁の多点ケース
# （design §4.1/§4.2/§4.5、ステップ7差し戻しイテレーション1）
# ========================================

def _make_object_names(n: int) -> list[str]:
    return [f"基準_{i:03d}" for i in range(n)]


def _write_dummy_image(path: Path, w: int = ORIG_W, h: int = ORIG_H) -> None:
    cv2.imwrite(str(path), np.zeros((h, w, 3), dtype=np.uint8))


class TestValidateCameraPointCount:
    """design §4.1 受け入れ基準2: 一意なObjectNameが「6個以上」でカメラ成立、
    6個以上なら部分選択せず全点を使用する（ちょうど6個固定からの変更）。
    """

    CAM = "camX"

    def _setup(self, tmp_path, n_points: int):
        names = _make_object_names(n_points)
        points_3d = {name: np.array([0.01 * i, 0.02 * i, 3.0], dtype=np.float64)
                     for i, name in enumerate(names)}
        # 2D座標は画像範囲内([0, W-1]x[0, H-1])に収める
        points_2d = {(self.CAM, name): np.array(
            [500.0 + 5 * i, 300.0 + 3 * i], dtype=np.float64)
            for i, name in enumerate(names)}
        img_path = tmp_path / f"{self.CAM}.png"
        _write_dummy_image(img_path)
        cam_toml = {"width": ORIG_W, "height": ORIG_H}
        return cam_toml, str(tmp_path), points_3d, points_2d, names

    def test_six_points_succeeds(self, tmp_path):
        cam_toml, images_dir, points_3d, points_2d, names = self._setup(tmp_path, 6)
        result = validate_camera(self.CAM, cam_toml, images_dir, points_3d, points_2d)
        assert result["ok"] is True
        assert len(result["names"]) == 6
        assert result["p3"].shape == (6, 3)
        assert result["p2"].shape == (6, 2)

    def test_five_points_fails_camera_unit(self, tmp_path):
        cam_toml, images_dir, points_3d, points_2d, names = self._setup(tmp_path, 5)
        result = validate_camera(self.CAM, cam_toml, images_dir, points_3d, points_2d)
        assert result["ok"] is False
        assert "6個以上" in result["reason"]

    def test_seven_points_succeeds_uses_all_points(self, tmp_path):
        cam_toml, images_dir, points_3d, points_2d, names = self._setup(tmp_path, 7)
        result = validate_camera(self.CAM, cam_toml, images_dir, points_3d, points_2d)
        assert result["ok"] is True
        assert result["names"] == sorted(names)
        assert len(result["names"]) == 7
        assert result["p3"].shape == (7, 3)
        assert result["p2"].shape == (7, 2)

    def test_twenty_points_succeeds_uses_all_points(self, tmp_path):
        cam_toml, images_dir, points_3d, points_2d, names = self._setup(tmp_path, 20)
        result = validate_camera(self.CAM, cam_toml, images_dir, points_3d, points_2d)
        assert result["ok"] is True
        assert result["names"] == sorted(names)
        assert len(result["names"]) == 20
        assert result["p3"].shape == (20, 3)
        assert result["p2"].shape == (20, 2)


class TestPnpManualMultiPoint:
    """design §4.2: 手動点PnPは入力された全手動点(N>=6)を使用する（6点だけの部分選択はしない）"""

    def test_ten_points_recovers_true_pose_and_uses_all_points(self):
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(5)
        rvec_true = np.zeros(3)
        tvec_true = np.array([0.0, 0.0, 5.0])

        xs = np.linspace(-0.6, 0.6, 5)
        ys = np.array([-0.3, 0.3])
        p3 = np.array([[x, y, 0.0] for x in xs for y in ys], dtype=np.float64)
        assert len(p3) == 10
        p2, _ = cv2.projectPoints(p3, rvec_true, tvec_true, K, D)
        p2 = p2.reshape(-1, 2)

        result = pnp_manual(p3, p2, K, D)
        assert result is not None
        rvec_est, tvec_est, inl = result
        # 全10点がノイズなしの整合データのためRANSAC inlierに全点が入る
        assert len(inl) == 10
        dpos, dang = pose_diff(rvec_est, tvec_est, rvec_true, tvec_true)
        assert dpos == pytest.approx(0.0, abs=1e-3)
        assert dang == pytest.approx(0.0, abs=1e-2)


class TestArbitrationMultiPoint:
    """design §4.5 仲裁: 「入力された全手動点」の再投影中央値で二峰を仲裁する。
    D3実証は多点手動点による仲裁のため、6点固定ではなく10点で検証する。
    """

    def test_bimodal_arbitration_with_ten_points_picks_lower_reprojection_peak(self):
        K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
        D = np.zeros(5)

        # 真の姿勢（第1クラスタ中心）と、0.5m平行移動したずれ姿勢（第2クラスタ中心）
        center1 = np.array([0.0, 0.0, -5.0])
        center2 = center1 + np.array([0.5, 0.0, 0.0])
        rvec1, tvec1 = _make_pose(center1)
        rvec2, tvec2 = _make_pose(center2)

        xs = np.linspace(-0.3, 0.3, 5)
        ys = np.array([-0.15, 0.15])
        p3 = np.array([[x, y, 0.0] for x in xs for y in ys], dtype=np.float64)
        assert len(p3) == 10
        p2, _ = cv2.projectPoints(p3, rvec1, tvec1, K, D)
        p2 = p2.reshape(-1, 2)

        # 手動点PnPの再投影誤差は第1クラスタ(真の姿勢)側が明確に小さい
        med1 = median_reproj_px(p3, p2, rvec1, tvec1, K, D)
        med2 = median_reproj_px(p3, p2, rvec2, tvec2, K, D)
        assert med1 == pytest.approx(0.0, abs=1e-6)
        assert med2 - med1 >= 1.0

        rng = np.random.default_rng(2)

        def _samples_near(center, n):
            out = []
            for _ in range(n):
                c = center + rng.normal(scale=0.001, size=3)
                out.append(_make_pose(c, axis_deg=rng.normal(scale=0.01, size=3)))
            return out

        samples_c1 = _samples_near(center1, 42)
        samples_c2 = _samples_near(center2, 18)
        chain_results = [
            {"accepted": True, "samples": samples_c1[:20]},
            {"accepted": True, "samples": samples_c1[20:40]},
            {"accepted": True, "samples": samples_c1[40:] + samples_c2},
        ]

        decision = decide_acceptance(chain_results, p3, p2, K, D)
        assert decision["accepted"] is True
        assert "仲裁成立の二峰" in decision["mode"]
        dpos, dang = pose_diff(decision["rvec"], decision["tvec"], rvec1, tvec1)
        assert dpos == pytest.approx(0.0, abs=0.01)
        assert dang == pytest.approx(0.0, abs=0.5)
