"""
feat-016: キーポイントのオクルージョン（深度による前後判定）のテスト

bpy/CUDA/c3d 非依存の純粋ロジック（座標変換・投影・カメラ深度・可視判定・描画）を
合成データで検証する。c3d/torch/gsplat は render_keypoints の関数内 import なので
本テストでは import 不要。実データ依存テストはファイル非存在時に skip する。
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import (
    ALPHA_THRESH,
    HALPE26_NAMES,
    c3d_to_calib,
    compute_keypoint_depth,
    compute_visibility,
    draw_overlay,
    extract_halpe26,
    project_keypoints,
)

# 実データパス（phase4/data/Blender、gitignore対象）
_DATA = Path(__file__).parent.parent / "phase4" / "data" / "Blender"
_TOML = _DATA / "Config_scene.toml"


# ========================================
# c3d_to_calib（合成データ）
# ========================================

class TestC3dToCalib:
    def test_single_point(self):
        """(px,py,pz) mm → (pz,px,py)*0.001 m"""
        out = c3d_to_calib(np.array([1000.0, 2000.0, 3000.0]))
        np.testing.assert_allclose(out, [3.0, 1.0, 2.0])

    def test_array_shape_preserved(self):
        """(N,3) 入力で軸入れ替え・スケールが各行に適用される"""
        pts = np.array([[1000.0, 2000.0, 3000.0], [0.0, -1000.0, 500.0]])
        out = c3d_to_calib(pts)
        assert out.shape == (2, 3)
        np.testing.assert_allclose(out[0], [3.0, 1.0, 2.0])
        np.testing.assert_allclose(out[1], [0.5, 0.0, -1.0])

    def test_multidim(self):
        """末尾軸が3なら任意の先行次元を保つ"""
        pts = np.zeros((4, 26, 3))
        assert c3d_to_calib(pts).shape == (4, 26, 3)


# ========================================
# compute_keypoint_depth（合成データ）
# ========================================

class TestComputeKeypointDepth:
    def test_identity_rotation(self):
        """恒等回転では深度 = t_z + X_z"""
        cam = {"rvec": np.zeros(3), "tvec": np.array([0.0, 0.0, 2.0])}
        X = np.array([[0.0, 0.0, 0.5], [1.0, -1.0, 1.5]])
        d = compute_keypoint_depth(X, cam)
        np.testing.assert_allclose(d, [2.5, 3.5])

    def test_matches_manual_rotation(self):
        """任意 rvec で (R·X+t)_z と一致"""
        rvec = np.array([0.3, -0.5, 0.2])
        tvec = np.array([0.1, 0.2, 1.0])
        cam = {"rvec": rvec, "tvec": tvec}
        X = np.array([0.4, -0.3, 0.7])
        R, _ = cv2.Rodrigues(rvec)
        expected = (R @ X + tvec)[2]
        np.testing.assert_allclose(compute_keypoint_depth(X, cam), expected)


# ========================================
# extract_halpe26（合成データ）
# ========================================

class TestExtractHalpe26:
    def _make_labels_data(self):
        # 26マーカーを逆順で並べ、抽出が名前ベースであることを検証
        labels = list(reversed(HALPE26_NAMES))
        data = np.arange(26 * 3, dtype=np.float64).reshape(26, 3)
        residual = np.zeros(26, dtype=np.float64)
        return labels, data, residual

    def test_reorders_to_halpe26(self):
        """C3D順がどうあれ HALPE26_NAMES 順に並ぶ"""
        labels, data, residual = self._make_labels_data()
        kpts, valid = extract_halpe26(labels, data, residual)
        assert kpts.shape == (26, 3) and valid.shape == (26,)
        # HALPE26_NAMES[0]="Hip" は labels（逆順）の末尾 idx=25
        np.testing.assert_allclose(kpts[0], data[25])
        assert valid.all()

    def test_negative_residual_invalid(self):
        """residual < 0 の点は valid False"""
        labels, data, residual = self._make_labels_data()
        residual[5] = -1.0  # labels[5] に対応するマーカー
        _, valid = extract_halpe26(labels, data, residual)
        # labels は逆順なので labels[5] = HALPE26_NAMES[20]
        assert not valid[HALPE26_NAMES.index(labels[5])]

    def test_nan_coord_invalid(self):
        """座標にNaNを含む点は valid False"""
        labels, data, residual = self._make_labels_data()
        data[3, 1] = np.nan
        _, valid = extract_halpe26(labels, data, residual)
        assert not valid[HALPE26_NAMES.index(labels[3])]

    def test_missing_marker_raises(self):
        """Halpe26名が不足すると ValueError（不足名を含む）"""
        labels = list(HALPE26_NAMES)
        labels[0] = "Unknown"  # "Hip" が欠ける
        data = np.zeros((26, 3))
        residual = np.zeros(26)
        with pytest.raises(ValueError, match="Hip"):
            extract_halpe26(labels, data, residual)


# ========================================
# compute_visibility（合成データ）
# ========================================

class TestComputeVisibility:
    def _maps(self, H=10, W=10, depth=5.0, alpha=1.0):
        depth_map = np.full((H, W), depth, dtype=np.float32)
        alpha_map = np.full((H, W), alpha, dtype=np.float32)
        return depth_map, alpha_map

    def test_invalid_false(self):
        """valid False → False（最優先）"""
        d_map, a_map = self._maps()
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([1.0]), np.array([False]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == False

    def test_behind_camera_false(self):
        """depth_cam <= near_plane（背面・至近）→ False"""
        d_map, a_map = self._maps()
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([0.05]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == False

    def test_nonfinite_depth_false(self):
        """depth_cam が NaN/inf → False"""
        d_map, a_map = self._maps()
        for bad in (np.nan, np.inf):
            vis = compute_visibility(
                np.array([[5.0, 5.0]]), np.array([bad]), np.array([True]),
                d_map, a_map, margin=0.05, near_plane=0.1)
            assert vis[0] == False

    def test_out_of_frame_true(self):
        """画像外 → True（隠す3DGSが画面内に無い）"""
        d_map, a_map = self._maps()
        vis = compute_visibility(
            np.array([[100.0, 100.0]]), np.array([3.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == True

    def test_low_alpha_true(self):
        """低α画素 → True（3DGSなし/薄い）"""
        d_map, a_map = self._maps(alpha=0.1)  # < ALPHA_THRESH
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([10.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == True

    def test_behind_3dgs_false(self):
        """3DGSより奥（depth_cam > depth_map + margin）→ False（隠れる）"""
        d_map, a_map = self._maps(depth=5.0, alpha=1.0)
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([6.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == False

    def test_in_front_true(self):
        """3DGSより手前 → True"""
        d_map, a_map = self._maps(depth=5.0, alpha=1.0)
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([4.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == True

    def test_alpha_before_depth(self):
        """判定順序: 低α画素では深度が奥でも隠さない（α判定が深度比較より先）"""
        d_map, a_map = self._maps(depth=1.0, alpha=0.1)  # 低α
        vis = compute_visibility(
            np.array([[5.0, 5.0]]), np.array([99.0]), np.array([True]),  # 深度は大きく奥
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == True  # 低α優先で可視

    def test_boundary_col_equals_width(self):
        """col==W は画像外扱い（< 比較）→ True"""
        d_map, a_map = self._maps(H=10, W=10)
        vis = compute_visibility(
            np.array([[10.0, 5.0]]), np.array([3.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == True

    def test_rounding_index(self):
        """(u,v) は round で画素参照。4.6→5 に丸めて深度比較が成立"""
        d_map, a_map = self._maps(depth=5.0, alpha=1.0)
        d_map[5, 5] = 1.0  # row=5,col=5 だけ手前に3DGS
        vis = compute_visibility(
            np.array([[4.6, 4.6]]), np.array([3.0]), np.array([True]),
            d_map, a_map, margin=0.05, near_plane=0.1)
        assert vis[0] == False  # round(4.6)=5 → depth_map[5,5]=1.0, 3.0>1.05 で隠れる


# ========================================
# draw_overlay（合成データ）
# ========================================

class TestDrawOverlay:
    def _setup(self):
        # 26点を画像中央付近に格子状に配置（全て画像内）
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        kpts_calib = np.zeros((26, 3), dtype=np.float64)
        pts2d = np.array([[10 + (i % 13) * 6, 10 + (i // 13) * 6] for i in range(26)],
                         dtype=np.float64)
        cam = {"rvec": np.zeros(3), "tvec": np.array([0.0, 0.0, 2.0]),
               "K": np.array([[100.0, 0, 50], [0, 100.0, 50], [0, 0, 1]])}
        return img, kpts_calib, pts2d, cam

    def test_no_crash_and_returns_copy(self):
        """例外なく描画し、入力画像を破壊しない"""
        img, kpts_calib, pts2d, cam = self._setup()
        valid = np.ones(26, dtype=bool)
        vis = np.ones(26, dtype=bool)
        out = draw_overlay(img, kpts_calib, pts2d, valid, vis, cam,
                           None, None, margin=0.05, near_plane=0.1, occlusion=False)
        assert out.shape == img.shape
        assert (img == 0).all()  # 元画像は不変
        assert (out != 0).any()  # 何か描かれた

    def test_invisible_points_not_drawn(self):
        """occlusion=True で kp_visible False の点は描かれない"""
        img, kpts_calib, pts2d, cam = self._setup()
        valid = np.ones(26, dtype=bool)
        vis_all = np.ones(26, dtype=bool)
        vis_none = np.zeros(26, dtype=bool)
        depth_map = np.full((100, 100), 100.0, dtype=np.float32)
        alpha_map = np.zeros((100, 100), dtype=np.float32)
        out_all = draw_overlay(img, kpts_calib, pts2d, valid, vis_all, cam,
                               depth_map, alpha_map, 0.05, 0.1, occlusion=True)
        out_none = draw_overlay(img, kpts_calib, pts2d, valid, vis_none, cam,
                                depth_map, alpha_map, 0.05, 0.1, occlusion=True)
        # 可視ゼロのほうが描画画素が少ない（点が消える）
        assert int((out_none != 0).sum()) < int((out_all != 0).sum())

    def test_invalid_points_not_drawn(self):
        """valid False の点は occlusion 無効でも描かれない"""
        img, kpts_calib, pts2d, cam = self._setup()
        valid_all = np.ones(26, dtype=bool)
        valid_none = np.zeros(26, dtype=bool)
        vis = np.ones(26, dtype=bool)
        out_all = draw_overlay(img, kpts_calib, pts2d, valid_all, vis, cam,
                               None, None, 0.05, 0.1, occlusion=False)
        out_none = draw_overlay(img, kpts_calib, pts2d, valid_none, vis, cam,
                                None, None, 0.05, 0.1, occlusion=False)
        assert (out_none == 0).all()  # 何も描かれない
        assert (out_all != 0).any()


# ========================================
# project_keypoints（実データ依存・skipif）
# ========================================

@pytest.mark.skipif(not _TOML.exists(), reason="実データTOMLが無い環境ではスキップ")
class TestProjectKeypointsReal:
    def test_projects_to_2d(self):
        """実TOMLの1カメラでキャリブ座標を投影し (N,2) を返す（歪みなし）"""
        from render_keypoints import load_cameras_toml, select_camera
        cams = load_cameras_toml(str(_TOML))
        cam = select_camera(cams, "cam41520554")
        # シーン原点付近の3点
        kpts = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0]])
        pts2d = project_keypoints(kpts, cam)
        assert pts2d.shape == (3, 2)
        assert np.isfinite(pts2d).all()
