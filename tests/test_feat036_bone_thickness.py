"""
feat-036: render_keypoints.py ボーン線の太さ変更のテスト

gsplat/torch を import しない範囲（定数・draw_overlay・draw_frustum）で合成データを使う。
c3d/torch/gsplat は render_keypoints の関数内 import なので本テストでは import 不要
（既存 feat-016/034 テストと同じ方針）。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import render_keypoints  # noqa: E402
from render_keypoints import (  # noqa: E402
    BONE_THICKNESS,
    FRUSTUM_THICKNESS,
    HALPE26_NAMES,
    KEYPOINT_NAMES,
    POINT_RADIUS,
    build_skeleton,
)


# ========================================
# T-01 定数値
# ========================================

def test_t01_constant_values():
    """BONE_THICKNESS=20, FRUSTUM_THICKNESS=2, POINT_RADIUS=4, LINE_THICKNESS は存在しない"""
    assert BONE_THICKNESS == 20
    assert FRUSTUM_THICKNESS == 2
    assert POINT_RADIUS == 4
    assert hasattr(render_keypoints, "LINE_THICKNESS") is False


# ========================================
# draw_overlay 用の合成データ（TestDrawOverlay._setup と同じ流儀）
# ========================================

def _overlay_setup():
    # 28点を画像中央付近に格子状に配置（全て画像内）
    n = len(KEYPOINT_NAMES)
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    kpts_calib = np.zeros((n, 3), dtype=np.float64)
    pts2d = np.array([[10 + (i % 14) * 6, 10 + (i // 14) * 6] for i in range(n)],
                     dtype=np.float64)
    cam = {"rvec": np.zeros(3), "tvec": np.array([0.0, 0.0, 2.0]),
           "K": np.array([[100.0, 0, 50], [0, 100.0, 50], [0, 0, 1]])}
    return img, kpts_calib, pts2d, cam


_OVERLAY_SKELETON = build_skeleton(set(HALPE26_NAMES))


def _install_line_spy(monkeypatch) -> list:
    """render_keypoints.cv2.line をラップし、呼び出しの thickness を recorded に記録する。"""
    recorded: list = []
    orig_line = render_keypoints.cv2.line

    def spy_line(img, pt1, pt2, color, thickness=1, *args, **kwargs):
        recorded.append(thickness)
        return orig_line(img, pt1, pt2, color, thickness, *args, **kwargs)

    monkeypatch.setattr(render_keypoints.cv2, "line", spy_line)
    return recorded


# ========================================
# T-02 draw_overlay occlusion=False のボーン線太さ
# ========================================

def test_t02_draw_overlay_occlusion_false_bone_thickness(monkeypatch):
    img, kpts_calib, pts2d, cam = _overlay_setup()
    valid = np.ones(len(KEYPOINT_NAMES), dtype=bool)
    kp_visible = np.ones(len(KEYPOINT_NAMES), dtype=bool)
    recorded = _install_line_spy(monkeypatch)

    render_keypoints.draw_overlay(
        img, kpts_calib, pts2d, valid, kp_visible, cam, _OVERLAY_SKELETON,
        None, None, margin=0.05, near_plane=0.1, occlusion=False,
    )

    assert len(recorded) > 0
    assert all(t == 20 for t in recorded)


# ========================================
# T-03 draw_overlay occlusion=True のボーン線太さ
# ========================================

def test_t03_draw_overlay_occlusion_true_bone_thickness(monkeypatch):
    img, kpts_calib, pts2d, cam = _overlay_setup()
    valid = np.ones(len(KEYPOINT_NAMES), dtype=bool)
    kp_visible = np.ones(len(KEYPOINT_NAMES), dtype=bool)
    depth_map = np.full((100, 100), 100.0, dtype=np.float32)
    alpha_map = np.zeros((100, 100), dtype=np.float32)  # 全サンプル可視（低α扱い）
    recorded = _install_line_spy(monkeypatch)

    render_keypoints.draw_overlay(
        img, kpts_calib, pts2d, valid, kp_visible, cam, _OVERLAY_SKELETON,
        depth_map, alpha_map, margin=0.05, near_plane=0.1, occlusion=True,
    )

    assert len(recorded) > 0
    assert all(t == 20 for t in recorded)


# ========================================
# draw_frustum 用の合成データ（test_feat034_fps_frustum.py と同じ流儀）
# ========================================

def _frustum_test_verts():
    """draw_frustum テスト用の5頂点（apex z=1、遠端矩形 z=2 の小さな四角錐）。"""
    return np.array([
        [0.0, 0.0, 1.0],
        [-0.3, -0.3, 2.0],
        [0.3, -0.3, 2.0],
        [0.3, 0.3, 2.0],
        [-0.3, 0.3, 2.0],
    ])


def _frustum_test_cam(W=200, H=100):
    return {
        "K": np.array([[100.0, 0.0, 100.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]]),
        "rvec": np.zeros(3),
        "tvec": np.zeros(3),
        "width": W, "height": H,
    }


# ========================================
# T-04 draw_frustum occlusion=False の線太さ
# ========================================

def test_t04_draw_frustum_occlusion_false_thickness(monkeypatch):
    H, W = 100, 200
    image = np.zeros((H, W, 3), dtype=np.uint8)
    cam = _frustum_test_cam(W, H)
    verts = _frustum_test_verts()
    recorded = _install_line_spy(monkeypatch)

    render_keypoints.draw_frustum(
        image, verts, cam, None, None, margin=0.05, near_plane=0.1, occlusion=False,
    )

    assert len(recorded) > 0
    assert all(t == 2 for t in recorded)


# ========================================
# T-05 draw_frustum occlusion=True の線太さ
# ========================================

def test_t05_draw_frustum_occlusion_true_thickness(monkeypatch):
    H, W = 100, 200
    image = np.zeros((H, W, 3), dtype=np.uint8)
    cam = _frustum_test_cam(W, H)
    verts = _frustum_test_verts()
    depth_map = np.full((H, W), 100.0, dtype=np.float32)
    alpha_map = np.ones((H, W), dtype=np.float32)  # 全サンプル手前（可視）
    recorded = _install_line_spy(monkeypatch)

    render_keypoints.draw_frustum(
        image, verts, cam, depth_map, alpha_map, margin=0.05, near_plane=0.1, occlusion=True,
    )

    assert len(recorded) > 0
    assert all(t == 2 for t in recorded)
