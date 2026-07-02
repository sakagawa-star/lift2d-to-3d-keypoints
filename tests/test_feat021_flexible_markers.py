"""
feat-021: render_keypoints.py 欠損マーカー許容（22点C3D対応）のテスト

bpy/CUDA/c3d 非依存の純粋ロジック（extract_keypoints, build_skeleton）と、
monkeypatch による main の早期エラー経路（FR-005）を合成データで検証する。
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

import render_keypoints
from render_keypoints import (
    HALPE26_NAMES,
    KEYPOINT_NAMES,
    build_skeleton,
    extract_keypoints,
)

# 22点C3D（session001_world_22pt 構成）のラベル
LABELS_22PT = [
    "Hip", "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle",
    "Spine", "Thorax", "Neck", "Head", "LShoulder", "LElbow", "LWrist",
    "RShoulder", "RElbow", "RWrist", "Nose", "LEye", "REye", "LEar", "REar",
]

# 足先6点（22点C3Dに存在しない既知マーカー）
FOOT_NAMES = ["RBigToe", "RSmallToe", "RHeel", "LBigToe", "LSmallToe", "LHeel"]

# 変更前の HALPE26_SKELETON（feat-016 実装の定数。25本・同順。
# 26点C3D（Spine/Thorax なし）で描画内容が変更前と同一であることの期待値）
OLD_HALPE26_SKELETON = [
    ("Head", "Nose", "C"), ("Nose", "Neck", "C"), ("Neck", "Hip", "C"),
    ("Nose", "REye", "R"), ("REye", "REar", "R"),
    ("Nose", "LEye", "L"), ("LEye", "LEar", "L"),
    ("Neck", "RShoulder", "R"), ("RShoulder", "RElbow", "R"), ("RElbow", "RWrist", "R"),
    ("Neck", "LShoulder", "L"), ("LShoulder", "LElbow", "L"), ("LElbow", "LWrist", "L"),
    ("Hip", "RHip", "R"), ("RHip", "RKnee", "R"), ("RKnee", "RAnkle", "R"),
    ("Hip", "LHip", "L"), ("LHip", "LKnee", "L"), ("LKnee", "LAnkle", "L"),
    ("RAnkle", "RHeel", "R"), ("RAnkle", "RBigToe", "R"), ("RBigToe", "RSmallToe", "R"),
    ("LAnkle", "LHeel", "L"), ("LAnkle", "LBigToe", "L"), ("LBigToe", "LSmallToe", "L"),
]


# ========================================
# extract_keypoints（合成データ）
# ========================================

class TestExtractKeypoints:
    def test_extract_keypoints_22pt(self):
        """22点C3D: 足先6点は valid=False、存在する22点は valid/座標が正しい（FR-001）"""
        n = len(LABELS_22PT)
        data = np.arange(n * 3, dtype=np.float64).reshape(n, 3)
        residual = np.zeros(n, dtype=np.float64)
        kpts, valid = extract_keypoints(LABELS_22PT, data, residual)

        assert kpts.shape == (28, 3) and valid.shape == (28,)
        for name in FOOT_NAMES:
            i = KEYPOINT_NAMES.index(name)
            assert not valid[i]
            np.testing.assert_allclose(kpts[i], 0.0)
        for name in LABELS_22PT:
            i = KEYPOINT_NAMES.index(name)
            assert valid[i]
            np.testing.assert_allclose(kpts[i], data[LABELS_22PT.index(name)])

    def test_extract_keypoints_26pt(self):
        """26点C3D（Halpe26完備）: 先頭26点が変更前 extract_halpe26 と同値（FR-001 基準2(b)）"""
        # 変更前と同じ検証条件: 逆順ラベルで名前ベース抽出を確認
        labels = list(reversed(HALPE26_NAMES))
        data = np.arange(26 * 3, dtype=np.float64).reshape(26, 3)
        residual = np.zeros(26, dtype=np.float64)
        residual[5] = -1.0  # labels[5] = HALPE26_NAMES[20] を無効サンプルに

        kpts, valid = extract_keypoints(labels, data, residual)

        # 変更前 extract_halpe26 の期待値: HALPE26_NAMES[i] は labels の idx=25-i
        for i in range(26):
            np.testing.assert_allclose(kpts[i], data[25 - i])
            assert valid[i] == (25 - i != 5)
        # Spine/Thorax は欠損
        assert not valid[26:].any()
        np.testing.assert_allclose(kpts[26:], 0.0)


# ========================================
# build_skeleton（純粋関数）
# ========================================

class TestBuildSkeleton:
    def test_build_skeleton_full28(self):
        """28点 present: 体幹が Neck–Thorax–Spine–Hip の3本、Neck–Hip 直結なし（FR-003）"""
        skeleton = build_skeleton(set(KEYPOINT_NAMES))
        pairs = [(a, b) for a, b, _ in skeleton]
        assert ("Neck", "Thorax") in pairs
        assert ("Thorax", "Spine") in pairs
        assert ("Spine", "Hip") in pairs
        assert ("Neck", "Hip") not in pairs
        assert len(skeleton) == 27  # 25本 - 体幹1本 + 体幹3本

    def test_build_skeleton_22pt(self):
        """22点 present: 足先ボーン6本が除外される（FR-002）"""
        skeleton = build_skeleton(set(LABELS_22PT))
        for _, end, _ in skeleton:
            assert end not in FOOT_NAMES
        for start, _, _ in skeleton:
            assert start not in FOOT_NAMES
        # 体幹以外24本 - 足先6本 = 18本、+ 体幹チェーン3本 = 21本
        assert len(skeleton) == 21

    def test_build_skeleton_halpe26(self):
        """Spine/Thorax なし26点 present: 変更前 HALPE26_SKELETON と完全一致（FR-001 基準2(a)）"""
        skeleton = build_skeleton(set(HALPE26_NAMES))
        assert skeleton == OLD_HALPE26_SKELETON

    def test_build_skeleton_thorax_only(self):
        """Thorax のみ: Neck–Thorax, Thorax–Hip の2本（FR-003）"""
        skeleton = build_skeleton(set(HALPE26_NAMES) | {"Thorax"})
        pairs = [(a, b) for a, b, _ in skeleton]
        assert ("Neck", "Thorax") in pairs
        assert ("Thorax", "Hip") in pairs
        assert ("Neck", "Hip") not in pairs
        assert not any("Spine" in p for p in pairs)

    def test_build_skeleton_spine_only(self):
        """Spine のみ: Neck–Spine, Spine–Hip の2本（FR-003）"""
        skeleton = build_skeleton(set(HALPE26_NAMES) | {"Spine"})
        pairs = [(a, b) for a, b, _ in skeleton]
        assert ("Neck", "Spine") in pairs
        assert ("Spine", "Hip") in pairs
        assert ("Neck", "Hip") not in pairs
        assert not any("Thorax" in p for p in pairs)

    def test_build_skeleton_torso_position(self):
        """体幹ボーンはベーススケルトンの Neck–Hip と同じ位置（描画順維持）"""
        skeleton = build_skeleton(set(KEYPOINT_NAMES))
        # 先頭2本は変更前と同じ、3〜5本目が体幹チェーン
        assert skeleton[0] == ("Head", "Nose", "C")
        assert skeleton[1] == ("Nose", "Neck", "C")
        assert skeleton[2] == ("Neck", "Thorax", "C")
        assert skeleton[3] == ("Thorax", "Spine", "C")
        assert skeleton[4] == ("Spine", "Hip", "C")
        assert skeleton[5] == ("Nose", "REye", "R")


# ========================================
# main の早期エラー経路（FR-005、monkeypatch）
# ========================================

class TestMainNoKnownMarkers:
    def test_main_no_known_markers(self, monkeypatch, capsys, tmp_path):
        """既知マーカー0個: PLY/torch ロード前に終了コード1・stderrメッセージ（FR-005）"""
        # カメラ選択を通すための最小TOML
        toml_path = tmp_path / "calib.toml"
        toml_path.write_text(
            "[cam01]\n"
            'name = "cam01"\n'
            "size = [ 100.0, 100.0]\n"
            "matrix = [ [ 100.0, 0.0, 50.0], [ 0.0, 100.0, 50.0], [ 0.0, 0.0, 1.0]]\n"
            "distortions = [ 0.0, 0.0, 0.0, 0.0]\n"
            "rotation = [ 0.0, 0.0, 0.0]\n"
            "translation = [ 0.0, 0.0, 0.0]\n",
            encoding="utf-8",
        )

        # C3D読み込みを既知マーカーなしのダミーに差し替え（c3dライブラリ・実ファイル不要）
        def fake_load(_path):
            labels = ["Foo", "Bar"]
            frames = [{"frame_no": 1,
                       "data": np.zeros((2, 3)),
                       "residual": np.zeros(2)}]
            return labels, frames, 30.0

        monkeypatch.setattr(render_keypoints, "load_c3d_all_frames", fake_load)
        # render（torch）未ロード状態にして「PLYロード前に終了」を検証可能にする
        monkeypatch.delitem(sys.modules, "render", raising=False)

        rc = render_keypoints.main([
            "dummy.ply", str(toml_path), "dummy.c3d", "--camera", "cam01",
        ])

        assert rc == 1
        captured = capsys.readouterr()
        assert "C3Dに既知マーカーが1つもありません" in captured.err
        assert "Foo, Bar" in captured.err
        assert "render" not in sys.modules  # PLY/torch ロード前に終了した
