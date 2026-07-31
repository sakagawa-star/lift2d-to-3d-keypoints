"""feat-026 M3-1: サンプリング型停止判定の検証

criteria_m3_1.md(locked 2026-07-31)に基づき、停止判定(更新量<ε)を廃した
サンプリング方式を検証する:
  k=0(緩ゲート)1回 → k≥1 条件で停止せず N_S=20 回反復して解を収集 →
  合意クラスタ(r_pos=2cm, r_ang=0.3°)→ 占有率 f_c≥0.7 で受理、
  最終解=クラスタ内平均、散らばり(RMS半径)・第2クラスタ占有率を診断値化。

対象: 5カメラ × P0/P1/P2(判定15系列)+ cam05520129 の P3/P4(情報系列)
負対照: cam05520129 のポーズ・レンダに対しクエリを cam41520557 の実写に
差し替えた1ケース(受理されないことを要求)。

反復処理は m3_run.run_iteration_real をそのまま使う(M3 と同一)。
RANSAC 乱数は反復 j ごとに cv2.setRNGSeed(seed_series + j) で決定的に変える。

実行方法(プロジェクトルートで):
    TORCH_CUDA_ARCH_LIST="9.0+PTX" uv run --project phase4 python \
        docs/issues/feat-026-render-match-extrinsic-refinement/experiments/m3_realquery/m3_1_sampling.py

出力: m3_1_result.txt(コミット対象) / m3_1_history.npz(ローカル)
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
A1_DIR = Path(__file__).resolve().parents[1] / "a1_synthetic"
sys.path.insert(0, str(ROOT / "phase4"))
sys.path.insert(0, str(A1_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from render import load_ply  # noqa: E402
from render_keypoints import load_cameras_toml, select_camera  # noqa: E402
from stage2_smoke_test import PLY_PATH, TOML_PATH  # noqa: E402
from stage3_perturb_match import perturb_pose  # noqa: E402
from m3_run import (  # noqa: E402  # M3 と同一の反復・定数を使う
    run_iteration_real, pose_diff, CAMERAS, SERIES, P95_THRESHOLDS, IMAGES_DIR,
)

OUT_DIR = Path(__file__).resolve().parent
HISTORY_NPZ = OUT_DIR / "m3_history.npz"

# criteria_m3_1.md §1 の確定値
N_S = 20            # サンプリング反復数
R_POS = 0.02        # 合意クラスタ半径(位置)[m]
R_ANG = 0.30        # 合意クラスタ半径(角度)[deg]
F_C_MIN = 0.7       # 受理に必要な占有率(14/20 以上)
F2_BIMODAL = 0.2    # 第2クラスタ占有率がこれ以上なら「二峰」
SAME_SOL_POS_M = 0.01   # 同一解判定(criteria.md §5.2 と同値)
SAME_SOL_ANG_DEG = 0.1
SEED_SAMP_BASE = 5000   # seed_series = SEED_SAMP_BASE + ci*100 + si*25(j=0..20 と重ならない間隔)
NEG_CONTROL_SEED = 9000

# 判定系列(P0/P1/P2)。cam05520129 のみ P3/P4 を情報系列として追加
TARGET_CAM = "cam05520129"
JUDGE_SERIES = ("P0", "P1", "P2")
INFO_SERIES = ("P3", "P4")


def rot_of(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return R


def center_of(rvec, tvec):
    R = rot_of(rvec)
    return -R.T @ np.asarray(tvec, dtype=np.float64).reshape(3)


def cluster_analysis(samples):
    """samples: [(rvec, tvec)] → (クラスタindex集合, f_c, f2, 中心解, RMS半径pos/ang)"""
    n = len(samples)
    Cs = np.array([center_of(r, t) for r, t in samples])
    Rs = [rot_of(r) for r, t in samples]
    dpos = np.linalg.norm(Cs[:, None, :] - Cs[None, :, :], axis=2)
    dang = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            cos_t = (np.trace(Rs[i].T @ Rs[j]) - 1.0) / 2.0
            dang[i, j] = dang[j, i] = np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0)))
    within = (dpos <= R_POS) & (dang <= R_ANG)
    counts = within.sum(axis=1)
    anchor = int(np.argmax(counts))
    cluster = np.where(within[anchor])[0]
    f_c = len(cluster) / n
    # 第2クラスタ: クラスタ外の残りで同じ操作
    rest = np.array([i for i in range(n) if i not in set(cluster.tolist())])
    if len(rest) > 0:
        counts2 = within[np.ix_(rest, rest)].sum(axis=1)
        f2 = counts2.max() / n
    else:
        f2 = 0.0
    # 中心解: 位置=クラスタ平均、回転=anchor 相対の回転ベクトル平均(角度≤0.3°で線形化が成立)
    C_mean = Cs[cluster].mean(axis=0)
    R_anchor = Rs[anchor]
    ws = []
    for i in cluster:
        w, _ = cv2.Rodrigues(R_anchor.T @ Rs[i])
        ws.append(w.flatten())
    w_mean = np.mean(ws, axis=0)
    dR, _ = cv2.Rodrigues(w_mean.reshape(3, 1))
    R_final = R_anchor @ dR
    rvec_final, _ = cv2.Rodrigues(R_final)
    tvec_final = -R_final @ C_mean
    # RMS 半径(中心解から)
    rms_pos = float(np.sqrt(np.mean(np.linalg.norm(Cs[cluster] - C_mean, axis=1) ** 2)))
    angs = []
    for i in cluster:
        cos_t = (np.trace(R_final.T @ Rs[i]) - 1.0) / 2.0
        angs.append(np.degrees(np.arccos(np.clip(cos_t, -1.0, 1.0))))
    rms_ang = float(np.sqrt(np.mean(np.array(angs) ** 2)))
    return cluster, f_c, f2, (rvec_final.flatten(), tvec_final.flatten()), rms_pos, rms_ang


def run_sampling_series(gaussians, cam, img_q_gray, rvec0, tvec0, seed_series, lines):
    """1系列: k=0 → k≥1 条件で N_S 回サンプリング。戻り値 dict"""
    rvec_cur = np.asarray(rvec0, dtype=np.float64).flatten()
    tvec_cur = np.asarray(tvec0, dtype=np.float64).flatten()
    cv2.setRNGSeed(seed_series)  # j=0(k=0 反復)
    ok, (rvec_cur, tvec_cur), diag = run_iteration_real(
        gaussians, cam, img_q_gray, rvec_cur, tvec_cur, 0)
    if not ok:
        lines.append(f"  j=0(k=0): 失敗({diag.get('fail')}) → 明示的失敗")
        return {"accepted": False, "fail": f"k=0: {diag.get('fail')}"}
    samples = []
    for j in range(1, N_S + 1):
        cv2.setRNGSeed(seed_series + j)
        ok, (rvec_new, tvec_new), diag = run_iteration_real(
            gaussians, cam, img_q_gray, rvec_cur, tvec_cur, 1)
        if not ok:
            lines.append(f"  j={j}: 失敗({diag.get('fail')}) → 明示的失敗")
            return {"accepted": False, "fail": f"j={j}: {diag.get('fail')}"}
        rvec_cur, tvec_cur = rvec_new, tvec_new
        samples.append((rvec_cur.copy(), tvec_cur.copy()))
    cluster, f_c, f2, (rvec_f, tvec_f), rms_pos, rms_ang = cluster_analysis(samples)
    accepted = f_c >= F_C_MIN
    lines.append(f"  サンプル{N_S}個: f_c={f_c:.2f}({len(cluster)}/{N_S}) 第2クラスタ={f2:.2f} "
                 f"RMS半径={rms_pos*1000:.2f}mm/{rms_ang:.4f}deg → "
                 f"{'受理' if accepted else '不受理(合意なし)'}"
                 + ("【二峰】" if f2 >= F2_BIMODAL else ""))
    return {"accepted": accepted, "f_c": f_c, "f2": f2,
            "rvec": rvec_f, "tvec": tvec_f, "rms_pos": rms_pos, "rms_ang": rms_ang,
            "samples": samples}


def main():
    hist = np.load(HISTORY_NPZ, allow_pickle=True)
    m3_results = hist["results"][0]

    print("PLY読み込み中...")
    gaussians = load_ply(PLY_PATH)
    cams_cfg = load_cameras_toml(TOML_PATH)

    lines = [
        "feat-026 M3-1: サンプリング型停止判定の検証(criteria_m3_1.md locked 2026-07-31)",
        f"N_S={N_S} r_pos={R_POS*100:.0f}cm r_ang={R_ANG}deg f_c≥{F_C_MIN}(={int(F_C_MIN*N_S)}/{N_S}) "
        f"第2クラスタ二峰閾値={F2_BIMODAL}",
        f"バージョン: OpenCV {cv2.__version__} / NumPy {np.__version__}",
        "",
    ]
    results = {}
    for ci, name in enumerate(CAMERAS):
        cam = select_camera(cams_cfg, name)
        rvec_base = np.asarray(cam["rvec"], dtype=np.float64).flatten()
        tvec_base = np.asarray(cam["tvec"], dtype=np.float64).flatten()
        img_q_gray = cv2.imread(str(IMAGES_DIR / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        if img_q_gray is None:
            raise FileNotFoundError(f"クエリ画像がない: {name}")
        series_names = JUDGE_SERIES + (INFO_SERIES if name == TARGET_CAM else ())
        lines.append(f"=== {name} ===")
        print(f"\n=== {name} ===")
        cam_res = {}
        for si, (sname, ppos, pang, pseed) in enumerate(SERIES):
            if sname not in series_names:
                continue
            seed_series = SEED_SAMP_BASE + ci * 100 + si * 25
            if ppos is None:
                rvec0, tvec0 = rvec_base, tvec_base
                lines.append(f"[{sname}] 摂動なし seed_series={seed_series}")
            else:
                rng = np.random.default_rng(pseed)
                rvec0, tvec0 = perturb_pose(rvec_base, tvec_base, ppos, pang, rng)
                lines.append(f"[{sname}] 摂動 {ppos*100:.0f}cm/{pang:.0f}deg "
                             f"(摂動seed={pseed}) seed_series={seed_series}")
            print(lines[-1])
            sres = run_sampling_series(gaussians, cam, img_q_gray, rvec0, tvec0,
                                       seed_series, lines)
            if sres["accepted"]:
                d_b, a_b = pose_diff(sres["rvec"], sres["tvec"], rvec_base, tvec_base)
                sres["diff_base"] = (d_b, a_b)
                lines.append(f"  最終解と基準ポーズの差: {d_b*100:.3f}cm/{a_b:.4f}deg")
                m3s = m3_results[name]["series"].get(sname)
                if m3s is not None:
                    d_m, a_m = pose_diff(sres["rvec"], sres["tvec"], m3s["rvec"], m3s["tvec"])
                    sres["diff_m3"] = (d_m, a_m)
                    lines.append(f"  M3 の解との差: {d_m*100:.3f}cm/{a_m:.4f}deg")
            print("\n".join(lines[-2:]))
            cam_res[sname] = sres
        lines.append("")
        results[name] = cam_res

    # 負対照: cam05520129 のポーズ・レンダ × cam41520557 のクエリ
    lines.append("=== 負対照(129 のレンダ × 557 のクエリ) ===")
    print(lines[-1])
    cam129 = select_camera(cams_cfg, TARGET_CAM)
    img_ctrl = cv2.imread(str(IMAGES_DIR / "cam41520557.png"), cv2.IMREAD_GRAYSCALE)
    neg = run_sampling_series(
        gaussians, cam129, img_ctrl,
        np.asarray(cam129["rvec"], dtype=np.float64).flatten(),
        np.asarray(cam129["tvec"], dtype=np.float64).flatten(),
        NEG_CONTROL_SEED, lines)
    lines.append(f"負対照の受理={neg['accepted']}(受理されないことが合格)")
    lines.append("")

    # 判定(criteria_m3_1.md §3 の5条件)
    rescued_ok = all(results[c][s]["accepted"]
                     for c, s in [("cam05520129", "P0"), ("cam05520129", "P2"),
                                  ("cam41520554", "P2")])
    acc_ok, reg_ok, same_ok, uni_ok = True, True, True, True
    for name in CAMERAS:
        th_p, th_a = P95_THRESHOLDS[name]
        finals = []
        for sname in JUDGE_SERIES:
            s = results[name][sname]
            m3s = m3_results[name]["series"].get(sname)
            m3_established = m3s is not None and m3s.get("established")
            if "f2" in s and s["f2"] >= F2_BIMODAL:
                uni_ok = False
            if not s["accepted"]:
                acc_ok = False
                if m3_established:  # M3 成立系列が非受理 = 退行
                    reg_ok = False
                continue
            d_b, a_b = s["diff_base"]
            if d_b > th_p or a_b > th_a:
                acc_ok = False
            if m3_established and "diff_m3" in s:
                d_m, a_m = s["diff_m3"]
                if d_m > R_POS or a_m > R_ANG:
                    reg_ok = False
            elif m3_established and "diff_m3" not in s:
                reg_ok = False
            finals.append((s["rvec"], s["tvec"]))
        if len(finals) == 3:
            for i in range(3):
                for j in range(i + 1, 3):
                    dp, da = pose_diff(*finals[i], *finals[j])
                    if dp > SAME_SOL_POS_M or da > SAME_SOL_ANG_DEG:
                        same_ok = False
        else:
            same_ok = False
    neg_ok = not neg["accepted"]
    uni_neg_ok = uni_ok and neg_ok
    go = rescued_ok and acc_ok and reg_ok and same_ok and uni_neg_ok
    lines += [
        "=== M3-1 判定(criteria_m3_1.md §3) ===",
        f"1. 救済(129 P0/P2・554 P2 受理): {'○' if rescued_ok else '×'}",
        f"2. 正確度(全15系列 p95 以内): {'○' if acc_ok else '×'}",
        f"3. 無退行(M3成立系列が受理・2cm/0.3°以内): {'○' if reg_ok else '×'}",
        f"4. 同一解(P0/P1/P2 相互 1cm/0.1°): {'○' if same_ok else '×'}",
        f"5. 単峰性・負対照: {'○' if uni_neg_ok else '×'}(二峰なし={uni_ok}, 負対照不受理={neg_ok})",
        f"総合: {'Go' if go else 'No-Go'}",
    ]
    print("\n".join(lines[-7:]))

    (OUT_DIR / "m3_1_result.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT_DIR / "m3_1_history.npz",
             results=np.array([results], dtype=object),
             neg=np.array([neg], dtype=object), go=go)
    print(f"保存: {OUT_DIR / 'm3_1_result.txt'}, {OUT_DIR / 'm3_1_history.npz'}")


if __name__ == "__main__":
    main()
