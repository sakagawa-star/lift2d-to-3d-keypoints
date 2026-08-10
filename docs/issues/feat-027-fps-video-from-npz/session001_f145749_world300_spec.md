# session001_f145749_world300.npz — Data Specification

## 1. File overview

| Item | Value |
|---|---|
| Full path | `/home/user/Documents/Epilepsy/epilepsy_data/E0014/json_3d/session001_f145749_world300.npz` |
| File size | 74.8 KB (numpy compressed) |
| Patient | E0014 |
| Recording date | 2015-09-08 (session start 11:31:39) |
| Source session | `09082015113139_..._1_001-1_topright_json` |
| Frame range | f145599 – f145898 (300 frames, center: **f145749**) |
| Duration | ≈ 10 s @ 30 fps |
| Joints | 22 (17 body + 5 face) |
| Coordinate system | **World** (metres) |
| NaN frames | 0 / 300 (all valid) |
| Face solvePnP | 9 / 300 (3%) — 残り 291 frames は body-prior で補完 |

---

## 2. Arrays (npz keys)

| key | shape | dtype | description |
|---|---|---|---|
| `x3d_world` | (300, 22, 3) | float32 | 3D keypoints in **world coordinates** (metres). Axis order: [x, y, z]. NaN-free in this clip. |
| `frame_ids` | (300,) | int32 | Original video frame index (0-origin, consecutive). Range: 145599–145898. |
| `pnp_ok` | (300,) | bool | `True` = face points estimated via solvePnP (higher accuracy). `False` = body-prior fallback used for face joints (17–21). |
| `joint_names` | (22,) | str | Joint name strings corresponding to axis-1 of x3d_world. See §3. |
| `center_frame` | scalar | int32 | The reference frame this clip was centred on: **145749**. |
| `coord_system` | scalar | str | Coordinate system identifier: `"world"`. |

---

## 3. Joint index

| # | name | group | x range (m) | y range (m) | z range (m) |
|---|---|---|---|---|---|
| 0 | Hip | body | [-0.023, 1.111] | [-1.005, -0.753] | [-0.435, -0.130] |
| 1 | RHip | body | [0.094, 1.045] | [-1.057, -0.831] | [-0.454, -0.162] |
| 2 | RKnee | body | [0.082, 0.414] | [-0.778, -0.514] | [-0.422, -0.037] |
| 3 | RAnkle | body | [0.043, 0.371] | [-0.837, -0.501] | [-0.504, -0.076] |
| 4 | LHip | body | [-0.111, 1.111] | [-0.962, -0.774] | [-0.441, -0.084] |
| 5 | LKnee | body | [-0.093, 0.600] | [-0.800, -0.577] | [-0.404, -0.014] |
| 6 | LAnkle | body | [-0.258, 0.383] | [-0.951, -0.617] | [-0.574, -0.085] |
| 7 | Spine | body | [-0.016, 1.005] | [-1.031, -0.742] | [-0.300, 0.089] |
| 8 | Thorax | body | [0.002, 0.970] | [-1.132, -0.698] | [-0.051, 0.482] |
| 9 | Neck | body | [-0.002, 0.835] | [-1.107, -0.697] | [0.017, 0.600] |
| 10 | Head | body | [0.055, 0.737] | [-1.047, -0.732] | [0.224, 0.864] |
| 11 | LShoulder | body | [-0.139, 1.159] | [-1.099, -0.635] | [-0.043, 0.563] |
| 12 | LElbow | body | [-0.227, 0.730] | [-1.060, -0.480] | [-0.249, 0.036] |
| 13 | LWrist | body | [-0.259, 0.420] | [-0.877, -0.593] | [-0.300, 0.128] |
| 14 | RShoulder | body | [0.144, 0.842] | [-1.173, -0.624] | [-0.060, 0.400] |
| 15 | RElbow | body | [0.227, 0.721] | [-1.122, -0.589] | [-0.264, -0.080] |
| 16 | RWrist | body | [0.095, 0.388] | [-0.939, -0.491] | [-0.314, 0.102] |
| 17 | Nose | face | [0.038, 0.874] | [-0.993, -0.798] | [0.092, 0.674] |
| 18 | LEye | face | [0.009, 0.858] | [-1.009, -0.788] | [0.124, 0.710] |
| 19 | REye | face | [0.062, 0.846] | [-1.007, -0.779] | [0.119, 0.694] |
| 20 | LEar | face | [-0.062, 0.836] | [-1.078, -0.743] | [0.119, 0.715] |
| 21 | REar | face | [0.086, 0.818] | [-1.083, -0.705] | [0.103, 0.667] |

---

## 4. Coordinate system

```
world座標系  (キャリブレーション基準点が原点)
  単位: メートル (m)
  axes: x = 水平, y = 垂直, z = 奥行き  ← キャリブ依存

── 変換チェーン ──────────────────────────────────

  OpenPose 2D (topright, 960×540)
      ↓  H36M-17 へ関節マッピング
  TCNRoot + TCN2Dto3D
      ↓  x3d_17 in cam01座標  (学習時の出力空間)
      ↓  R_cam01.T @ (x − tvec_cam01)
  x3d_world  ← このファイルの座標系
      ↓  R_camX @ x_world + tvec_camX  (任意カメラへ再投影可)
  各カメラ座標 / 2D投影

── cam01 → world 変換パラメータ ─────────────────

  RVEC_cam01 = [0.5621, -2.3480,  1.5653]   (Rodrigues)
  TVEC_cam01 = [-0.3287,  0.3549,  1.7323]  (m)

  R_cam01 =
    [[-0.8903, -0.4554, -0.0042],
     [-0.1712,  0.3434, -0.9235],
     [ 0.4220, -0.8214, -0.3837]]

  x_world = R_cam01.T @ (x_cam01 − TVEC_cam01)
  x_cam01 = R_cam01  @ x_world  + TVEC_cam01
```

---

## 5. Face joints (index 17–21)

| Item | Detail |
|---|---|
| 推定方法 | 2段階: solvePnP → body-prior fallback |
| solvePnP | OpenPose 顔2D点（信頼度>0.3）が1点以上ある場合、topright カメラ (osakaE0014) を中間座標系として頭部姿勢を最適化。このクリップでは 9/300 frames。 |
| body-prior | Neck/Head/LShoulder/RShoulder から頭部座標系を構築し、学習済み顔テンプレート (face_template_oosaka.npz) を当てはめる。このクリップでは 291/300 frames。 |
| クリッピング | solvePnP 後に Nose-Neck > 183 mm または Nose-Head > 212 mm の場合 body-prior に差し替え、`pnp_ok=False` に。 |
| 精度 | solvePnP フレーム > body-prior フレーム。body-prior は頭部の向きに対して数 cm 程度の誤差が生じうる。 |

---

## 6. ECoG との時刻同期

| Item | Detail |
|---|---|
| ECoG ファイル | `E0014/aligned_eeg/{run}_aligned_eeg/` |
| video_times.npy | ECoG サンプルごとの「動画内時刻 (s)」。shape = (N_ecog,), fs = 1000 Hz |
| 対応付け | `frame_time = frame_ids / 30.0 (s)` → `np.searchsorted(video_times, frame_time)` で ECoG インデックスを取得 |
| このクリップの動画時刻 | 145599/30 = 4853.3 s 〜 145898/30 = 4863.3 s（セッション開始から約 80.9 分） |

---

## 7. Python load example

```python
import numpy as np

d = np.load("session001_f145749_world300.npz", allow_pickle=True)

x      = d["x3d_world"]           # (300, 22, 3)  float32  metres
fids   = d["frame_ids"]           # (300,)        int32    video frame index
pnp    = d["pnp_ok"]              # (300,)        bool     face quality flag
names  = d["joint_names"].tolist() # list[str] length 22

# joint へのアクセス例
nose_traj = x[:, 17, :]           # (300, 3)  Nose の軌跡
hip_traj  = x[:,  0, :]           # (300, 3)  Hip  の軌跡

# solvePnP フレームのみ使う場合
x_pnp = x[pnp]                    # (9, 22, 3)

# world → cam01 へ逆変換
import cv2
RVEC = np.array([0.5621205297727883, -2.347986142089513, 1.5653210311470378])
TVEC = np.array([-0.32867092015457694, 0.3549114029420098, 1.732344119530507])
R, _ = cv2.Rodrigues(RVEC)
x_cam01 = (R @ x.reshape(-1, 3).T).T + TVEC
x_cam01 = x_cam01.reshape(300, 22, 3)
```

---

## 8. Related files

| File | Description |
|---|---|
| `E0014/json_3d/…001-1_topright_json_22pt.npz` | Full session, cam01座標, shape (194279, 22, 3) |
| `E0014/json_3d/…001-1_topright_json_3d.npz` | Body only (17点), cam01座標, conf17 含む |
| `E0014/json/{session}/{prefix}_{fid:06d}.json` | 元の OpenPose 2D JSON (HALPE-26) |
| `E0014/aligned_eeg/09082015…_run_*_aligned_eeg/` | ECoG データ (55ch, 1000 Hz) + 時刻同期 |
| `2Dto3D/oosaka1h/…/face_template_oosaka.npz` | 顔テンプレート: face_local (5,3), template_local (7,3) |
