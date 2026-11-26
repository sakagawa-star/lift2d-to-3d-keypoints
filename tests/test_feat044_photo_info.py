"""feat-044: 写真情報ファイルの書き出しと前処理の読み込みのテスト（GPU・実データ不要）。

設計: docs/issues/feat-044-remove-expect-exif-options/design.md §6.2
"""
import os
import sys
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(Path(__file__).parent))

import extract_photo_info as epi  # noqa: E402
import prepare_phone_photo as ppp  # noqa: E402
from refine_extrinsics import InputError  # noqa: E402
from test_feat043_shared_yaml import ALL_KEYS, _yaml  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402

EXPECTED_TEXT = ("# 写真情報ファイル（extract_photo_info.py が Exif から書き出し。feat-044）\n"
                 "focal_length_35mm: 23\n")


def _fake_focal(monkeypatch, value):
    monkeypatch.setattr(epi, "read_focal_35mm", lambda path: value)


def _write_pi(tmp_path, text="focal_length_35mm: 24\n", name="photo_info.yaml") -> Path:
    p = tmp_path / name
    p.write_text(text, encoding="utf-8")
    return p


def _no_exif(monkeypatch):
    monkeypatch.setattr(ppp, "read_exif", lambda path: {
        "Orientation": None, "DigitalZoomRatio": None, "Size": (64, 48)})


def _prep_inputs(tmp_path):
    src = tmp_path / "orig.jpg"
    cv2.imwrite(str(src), np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8))
    csv_in = tmp_path / "p_in.csv"
    lines = ["ObjectName,camera_name,X,Y"] + [f"P{k},camA,{10 + k * 8},{12 + k * 4}" for k in range(6)]
    csv_in.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return src, csv_in


def _prep_argv(src, csv_in, pi, images_dir, init_toml, csv_out, *extra):
    return ["--image", str(src), "--photo-info", str(pi), "--camera", "camA",
            "--images-dir", str(images_dir), "--init-toml", str(init_toml),
            "--points-2d-original", str(csv_in), "--points-2d", str(csv_out), *extra]


def test_1_extract_ok(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    out = tmp_path / "pi.yaml"
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 0
    assert out.read_text(encoding="utf-8") == EXPECTED_TEXT
    assert not Path(str(out) + ".tmp").exists()
    assert sorted(p.name for p in tmp_path.iterdir()) == ["pi.yaml"]
    assert ppp.load_photo_info(str(out)) == 23.0


@pytest.mark.parametrize("value", [None, 0])
def test_2_extract_no_value(value, monkeypatch, tmp_path, capsys):
    _fake_focal(monkeypatch, value)
    out = tmp_path / "pi.yaml"
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 1
    assert not out.exists()
    assert "focal_length_35mm:" in capsys.readouterr().err


def test_3_extract_exists(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    out = tmp_path / "pi.yaml"
    out.write_text("x", encoding="utf-8")
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 1
    assert out.read_text(encoding="utf-8") == "x"


def test_4_extract_open_error(monkeypatch, tmp_path):
    def boom(path):
        raise OSError("cannot open")

    monkeypatch.setattr(epi, "read_focal_35mm", boom)
    out = tmp_path / "pi.yaml"
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 1
    assert not out.exists()


def test_5_extract_empty_path(monkeypatch):
    _fake_focal(monkeypatch, 23)
    with pytest.raises(SystemExit) as ei:
        epi.main(["--image", "x.jpg", "--photo-info", ""])
    assert ei.value.code == 2


def test_6_extract_shared_yaml(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    out = tmp_path / "pi.yaml"
    conf = _yaml(tmp_path / "job.yaml", {**ALL_KEYS, "photo_info": out})
    assert epi.main(["--config", conf]) == 0
    assert out.read_text(encoding="utf-8") == EXPECTED_TEXT


def test_7_extract_abbrev(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    with pytest.raises(SystemExit) as ei:
        epi.main(["--image", "x.jpg", "--photo", str(tmp_path / "pi.yaml")])
    assert ei.value.code == 2


def test_8_load_photo_info(tmp_path):
    assert ppp.load_photo_info(str(_write_pi(tmp_path, "focal_length_35mm: 23\n", "a.yaml"))) == 23.0
    assert ppp.load_photo_info(str(_write_pi(tmp_path, "focal_length_35mm: 23.5\n", "b.yaml"))) == 23.5
    assert ppp.load_photo_info(
        str(_write_pi(tmp_path, "# コメント\nfocal_length_35mm: 23\n", "c.yaml"))) == 23.0


@pytest.mark.parametrize("text", [None, "", "foo: 1\nfocal_length_35mm: 23\n",
                                  "focal_length_35mm: abc\n", "focal_length_35mm: 0\n",
                                  "focal_length_35mm: -1\n", "focal_length_35mm: inf\n",
                                  "focal_length_35mm: nan\n"])
def test_9_load_photo_info_invalid(text, tmp_path):
    p = tmp_path / "missing.yaml" if text is None else _write_pi(tmp_path, text)
    with pytest.raises(InputError):
        ppp.load_photo_info(str(p))


def test_10_prepare_without_exif(monkeypatch, tmp_path):
    _no_exif(monkeypatch)
    src, csv_in = _prep_inputs(tmp_path)
    pi = _write_pi(tmp_path)
    init_toml = tmp_path / "init.toml"
    code = ppp.main(_prep_argv(src, csv_in, pi, tmp_path / "imgs", init_toml, tmp_path / "out.csv"))
    assert code == 0
    cam = select_camera(load_cameras_toml(str(init_toml)), "camA")
    assert abs(cam["K"][0, 0] - 24 * 80 / 43.27 * 30) < 1e-6


def test_11_prepare_invalid_photo_info(monkeypatch, tmp_path):
    _no_exif(monkeypatch)
    src, csv_in = _prep_inputs(tmp_path)
    pi = _write_pi(tmp_path, "foo: 1\n")
    outs = (tmp_path / "imgs", tmp_path / "init.toml", tmp_path / "out.csv")
    assert ppp.main(_prep_argv(src, csv_in, pi, *outs)) == 1
    for o in outs:
        assert not o.exists()


def _fake_pil(monkeypatch, ifd, size=(4000, 3000)):
    class _Exif(dict):
        def get_ifd(self, tag):
            return ifd

    class _Im:
        def __init__(self):
            self.size = size

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def getexif(self):
            return _Exif()

    image = types.ModuleType("PIL.Image")
    image.open = lambda path: _Im()
    pil = types.ModuleType("PIL")
    pil.Image = image
    monkeypatch.setitem(sys.modules, "PIL", pil)
    monkeypatch.setitem(sys.modules, "PIL.Image", image)


@pytest.mark.parametrize("value,expected", [(23, 23), (23.0, 23), (float("inf"), None),
                                            ("abc", None), ("NONE", None)])
def test_12_read_focal_35mm(value, expected, monkeypatch):
    ifd = {} if value == "NONE" else {epi._TAG_FOCAL_35MM: value}
    _fake_pil(monkeypatch, ifd)
    assert epi.read_focal_35mm("x.jpg") == expected


def test_13_existing_file_as_output(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    img = tmp_path / "orig.jpg"
    img.write_text("x", encoding="utf-8")
    assert epi.main(["--image", str(img), "--photo-info", str(img)]) == 1
    assert img.read_text(encoding="utf-8") == "x"
    job = tmp_path / "job.yaml"
    job.write_text(f"image: {img}\nphoto_info: {job}\n", encoding="utf-8")
    before = job.read_text(encoding="utf-8")
    assert epi.main(["--config", str(job)]) == 1
    assert job.read_text(encoding="utf-8") == before
    link = tmp_path / "link.yaml"
    os.symlink(img, link)
    assert epi.main(["--image", str(img), "--photo-info", str(link)]) == 1
    assert img.read_text(encoding="utf-8") == "x"


def test_14_tmp_symlink_not_followed(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    img = tmp_path / "orig.jpg"
    img.write_text("x", encoding="utf-8")
    out = tmp_path / "sub" / "pi.yaml"
    out.parent.mkdir()
    os.symlink(img, Path(str(out) + ".tmp"))
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 0
    assert out.read_text(encoding="utf-8") == EXPECTED_TEXT
    assert img.read_text(encoding="utf-8") == "x"
    assert not [p for p in out.parent.iterdir() if p.name.startswith(f".{out.name}.")]


def test_15_link_failure_leaves_nothing(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)

    def boom(*a, **k):
        raise OSError("no link")

    monkeypatch.setattr(epi.os, "link", boom)
    out = tmp_path / "pi.yaml"
    assert epi.main(["--image", "x.jpg", "--photo-info", str(out)]) == 1
    assert not out.exists()
    assert list(tmp_path.iterdir()) == []


def test_16_extract_overwrite_rejected(monkeypatch, tmp_path):
    _fake_focal(monkeypatch, 23)
    with pytest.raises(SystemExit) as ei:
        epi.main(["--image", "x.jpg", "--photo-info", str(tmp_path / "pi.yaml"), "--overwrite"])
    assert ei.value.code == 2


def test_17_read_exif_ignores_focal_tag(monkeypatch):
    _fake_pil(monkeypatch, {ppp._TAG_FOCAL_35MM: float("inf"), ppp._TAG_ORIENTATION: 1})
    r = ppp.read_exif("x.jpg")
    assert set(r) == {"Orientation", "DigitalZoomRatio", "Size"}
    assert "FocalLengthIn35mmFilm" not in r


def test_18_collision_with_outputs(monkeypatch, tmp_path):
    _no_exif(monkeypatch)
    src, csv_in = _prep_inputs(tmp_path)
    imgs = tmp_path / "imgs"
    csv_out = tmp_path / "out.csv"
    # (a) --init-toml と同じパス・--overwrite あり
    pi_a = _write_pi(tmp_path, name="pi_a.yaml")
    before = pi_a.read_text(encoding="utf-8")
    assert ppp.main(_prep_argv(src, csv_in, pi_a, imgs, pi_a, csv_out, "--overwrite")) == 1
    assert pi_a.read_text(encoding="utf-8") == before
    # (b) {init_toml}.tmp と同じパス・--overwrite なし
    init_b = tmp_path / "init_b.toml"
    pi_b = _write_pi(tmp_path, name="init_b.toml.tmp")
    assert ppp.main(_prep_argv(src, csv_in, pi_b, imgs, init_b, csv_out)) == 1
    assert pi_b.read_text(encoding="utf-8") == before
    # (c) --init-toml のパスに写真情報ファイルへのハードリンク・--overwrite あり
    pi_c = _write_pi(tmp_path, name="pi_c.yaml")
    init_c = tmp_path / "init_c.toml"
    os.link(pi_c, init_c)
    assert ppp.main(_prep_argv(src, csv_in, pi_c, imgs, init_c, csv_out, "--overwrite")) == 1
    assert pi_c.read_text(encoding="utf-8") == before
    assert not imgs.exists() and not csv_out.exists()
