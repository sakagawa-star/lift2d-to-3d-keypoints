"""
feat-024: render_keypoints.py 歪みモデル対応レンダリング（GT比較用）のテスト

GPU 不要の純ロジックテストのみ（design.md テスト設計 T-1〜T-10）。
"""

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "phase4"))

from render_keypoints import _build_parser, distortions_to_gsplat, main


_SYNTH_TOML = """\
[camA]
name = "camA"
size = [64, 48]
matrix = [[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]]
distortions = [0.1, -0.05, 0.001, 0.002, 0.0]
rotation = [0.0, 0.0, 0.0]
translation = [0.0, 0.0, 2.0]
"""


def _write_synth_toml(tmp_path) -> str:
    p = tmp_path / "synth.toml"
    p.write_text(_SYNTH_TOML)
    return str(p)


# ========================================
# T-1, T-2: distortions_to_gsplat
# ========================================

class TestDistortionsToGsplat:
    def test_len4(self):
        D = np.array([0.1, -0.05, 0.001, 0.002], dtype=np.float64)
        radial, tangential = distortions_to_gsplat(D)
        assert np.allclose(radial, [0.1, -0.05, 0.0, 0.0, 0.0, 0.0])
        assert np.allclose(tangential, [0.001, 0.002])

    def test_len5(self):
        D = np.array([0.1, -0.05, 0.001, 0.002, 0.01], dtype=np.float64)
        radial, tangential = distortions_to_gsplat(D)
        assert np.allclose(radial, [0.1, -0.05, 0.01, 0.0, 0.0, 0.0])
        assert np.allclose(tangential, [0.001, 0.002])

    def test_len8(self):
        D = np.array([0.1, -0.05, 0.001, 0.002, 0.01, 0.02, 0.03, 0.04], dtype=np.float64)
        radial, tangential = distortions_to_gsplat(D)
        assert np.allclose(radial, [0.1, -0.05, 0.01, 0.02, 0.03, 0.04])
        assert np.allclose(tangential, [0.001, 0.002])

    @pytest.mark.parametrize("n", [0, 3, 6, 9])
    def test_invalid_length(self, n):
        D = np.zeros(n, dtype=np.float64)
        with pytest.raises(ValueError) as excinfo:
            distortions_to_gsplat(D)
        assert str(n) in str(excinfo.value)


# ========================================
# T-3〜T-6: バリデーション
# ========================================

class TestValidation:
    def test_c3d_omitted_without_no_keypoints_exits_2(self, capsys, tmp_path):
        toml = _write_synth_toml(tmp_path)
        with pytest.raises(SystemExit) as excinfo:
            main(["a.ply", toml, "--camera", "camA"])
        assert excinfo.value.code == 2
        err = capsys.readouterr().err
        assert "--no-keypoints" in err

    def test_c3d_given_with_no_keypoints_exits_2(self, capsys, tmp_path):
        toml = _write_synth_toml(tmp_path)
        with pytest.raises(SystemExit) as excinfo:
            main(["a.ply", toml, "c.c3d", "--camera", "camA", "--no-keypoints"])
        assert excinfo.value.code == 2
        err = capsys.readouterr().err
        assert "c3d_path" in err

    @pytest.mark.parametrize("extra_args,opt_name", [
        (["--mp4"], "--mp4"),
        (["--mp4-fps", "30"], "--mp4-fps"),
        (["--no-png"], "--no-png"),
        (["--start-frame", "1"], "--start-frame"),
        (["--end-frame", "10"], "--end-frame"),
        (["--no-occlusion"], "--no-occlusion"),
        (["--occlusion-margin", "0.05"], "--occlusion-margin"),
    ])
    def test_still_mode_forbidden_options(self, capsys, tmp_path, extra_args, opt_name):
        toml = _write_synth_toml(tmp_path)
        with pytest.raises(SystemExit) as excinfo:
            main(["a.ply", toml, "--camera", "camA", "--no-keypoints"] + extra_args)
        assert excinfo.value.code == 2
        err = capsys.readouterr().err
        assert opt_name in err

    def test_distort_with_video_mode_exits_2(self, capsys, tmp_path):
        toml = _write_synth_toml(tmp_path)
        with pytest.raises(SystemExit) as excinfo:
            main(["a.ply", toml, "c.c3d", "--camera", "camA", "--distort"])
        assert excinfo.value.code == 2
        err = capsys.readouterr().err
        assert "--distort" in err


# ========================================
# T-7, T-8: render_image のフェイク torch/gsplat 注入
# ========================================

def _install_fake_torch_gsplat(monkeypatch):
    """render_image 内の `import torch` / `from gsplat import rasterization` を
    フェイクモジュールに差し替え、rasterization に渡された kwargs を記録する。

    Returns:
        captured: dict。rasterization 呼び出し後の kwargs が入る
    """
    captured = {}

    fake_torch = types.ModuleType("torch")
    fake_torch.float32 = "float32"

    class _FakeTensor:
        def __init__(self, data, device=None, dtype=None):
            self.data = data
            self.device = device
            self.dtype = dtype

        def unsqueeze(self, dim):
            return self

        def to(self, device):
            self.device = device
            return self

        def __getitem__(self, item):
            return self

        # 演算はテストで直接使わないため未実装のままでよい

    def _tensor(data, dtype=None, device=None):
        return _FakeTensor(data, device=device, dtype=dtype)

    def _from_numpy(arr):
        return _FakeTensor(arr)

    def _cat(tensors, dim=0):
        return _FakeTensor(tensors)

    class _NoGrad:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    fake_torch.tensor = _tensor
    fake_torch.from_numpy = _from_numpy
    fake_torch.cat = _cat
    fake_torch.no_grad = _NoGrad

    class _FakeRendered:
        """rasterization の戻り値 rendered をシミュレートする最小 (H,W,C) ラッパー。"""

        def __init__(self, H, W, C):
            self._data = np.zeros((1, H, W, C), dtype=np.float32)

        def squeeze(self, dim):
            return self._Squeezed(self._data[0])

        class _Squeezed:
            def __init__(self, arr):
                self._arr = arr

            def __getitem__(self, item):
                return self._Squeezed2(self._arr[item])

            class _Squeezed2:
                def __init__(self, arr):
                    self._arr = arr

                def cpu(self):
                    return self

                def numpy(self):
                    return self._arr

                def clamp(self, lo, hi):
                    return self

                def astype(self, dtype):
                    return self

    def _fake_rasterization(**kwargs):
        captured.update(kwargs)
        H, W = kwargs["height"], kwargs["width"]
        rendered = _FakeRendered(H, W, 4)
        alphas = _FakeRendered(H, W, 1)
        return rendered, alphas, {}

    fake_gsplat = types.ModuleType("gsplat")
    fake_gsplat.rasterization = _fake_rasterization

    monkeypatch.setitem(sys.modules, "torch", fake_torch)
    monkeypatch.setitem(sys.modules, "gsplat", fake_gsplat)

    return captured


def _dummy_cam():
    return {
        "name": "camA",
        "K": np.eye(3, dtype=np.float64),
        "D": np.array([0.1, -0.05, 0.001, 0.002], dtype=np.float64),
        "rvec": np.zeros(3, dtype=np.float64),
        "tvec": np.array([0.0, 0.0, 2.0], dtype=np.float64),
        "width": 64,
        "height": 48,
    }


def _dummy_gaussians():
    class _Tensor:
        def __init__(self, shape, device="cuda:0"):
            self.shape = shape
            self.device = device

    return {
        "means": _Tensor((10, 3)),
        "quats": _Tensor((10, 4)),
        "scales": _Tensor((10, 3)),
        "opacities": _Tensor((10,)),
        "sh0": _Tensor((10, 1, 3)),
        "sh_rest": _Tensor((10, 0, 3)),
    }


class TestRenderImageArgs:
    def test_default_distort_false_matches_current_behavior(self, monkeypatch):
        """T-7: distort引数なしで呼ぶと with_ut=False, packed=True, 歪み係数なし"""
        # render_keypoints は cv2 の演算（+,*,clamp等）も使うため、その部分を通さないために
        # render_image をそのまま呼ぶには numpy 変換に到達する必要がある。
        # ここでは cv2 合成部分に到達する前に rasterization 呼び出し引数のみを検証したいため、
        # rendered/alphas の numpy() 戻り値をそのまま使える形にする。
        captured = _install_fake_torch_gsplat(monkeypatch)
        import render_keypoints

        cam = _dummy_cam()
        gaussians = _dummy_gaussians()
        try:
            render_keypoints.render_image(gaussians, cam, near_plane=0.1)
        except Exception:
            # 背景合成以降（bg計算・BGR変換）はフェイクtorchでは完全に模倣できないため、
            # rasterization 呼び出し自体が成功していれば以降の例外は許容する
            pass
        assert captured.get("with_ut") is False
        assert captured.get("packed") is True
        assert captured.get("camera_model") == "pinhole"
        assert "radial_coeffs" not in captured
        assert "tangential_coeffs" not in captured

    def test_distort_true(self, monkeypatch):
        """T-8: distort=True で with_ut=True, with_eval3d=True, packed=False と係数の値"""
        captured = _install_fake_torch_gsplat(monkeypatch)
        import render_keypoints

        cam = _dummy_cam()
        gaussians = _dummy_gaussians()
        expected_radial, expected_tangential = distortions_to_gsplat(cam["D"])
        try:
            render_keypoints.render_image(gaussians, cam, near_plane=0.1, distort=True)
        except Exception:
            pass
        assert captured.get("with_ut") is True
        assert captured.get("with_eval3d") is True
        assert captured.get("packed") is False
        radial_arg = captured.get("radial_coeffs")
        tangential_arg = captured.get("tangential_coeffs")
        assert radial_arg is not None and tangential_arg is not None
        assert np.allclose(radial_arg.data[0], expected_radial)
        assert np.allclose(tangential_arg.data[0], expected_tangential)


# ========================================
# T-9: 静止画モードの出力ファイル名
# ========================================

class TestStillModeOutput:
    def test_still_mode_output_filename(self, monkeypatch, tmp_path):
        """_run_still_mode をモック環境で実行し still_<カメラ名>.png が出力される"""
        import render_keypoints

        dummy = types.ModuleType("render")
        dummy.load_ply = lambda path: {"means": "stub"}
        dummy.print_ply_summary = lambda path, gaussians: None
        monkeypatch.setitem(sys.modules, "render", dummy)

        H, W = 48, 64
        monkeypatch.setattr(
            "render_keypoints.render_image",
            lambda gaussians, cam, near_plane, background=(0.0, 0.0, 0.0), distort=False:
                np.zeros((H, W, 3), dtype=np.uint8),
        )
        imwrite = MagicMock(return_value=True)
        monkeypatch.setattr("render_keypoints.cv2.imwrite", imwrite)

        toml = _write_synth_toml(tmp_path)
        cameras = render_keypoints.load_cameras_toml(toml)
        cam = render_keypoints.select_camera(cameras, "camA")

        out_dir = tmp_path / "out"
        args = _build_parser().parse_args(
            ["a.ply", toml, "--camera", "camA", "--no-keypoints",
             "--output-dir", str(out_dir)]
        )
        if args.occlusion_margin is None:
            args.occlusion_margin = render_keypoints.OCCLUSION_MARGIN

        rc = render_keypoints._run_still_mode(args, cam)
        assert rc == 0
        assert imwrite.call_count == 1
        png_path = imwrite.call_args[0][0]
        assert png_path == str(out_dir / "still_camA.png")


# ========================================
# T-10: 既存テストの回帰なし（feat-015/016/017/021/022）は
# `uv run pytest -v` の全件実行で確認する（このファイル内には含めない）
# ========================================
