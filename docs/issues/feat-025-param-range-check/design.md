# 機能設計書: feat-025 estimate_camera_params.py 推定パラメータの範囲チェック（警告出力）

## 1.1 対応要求マッピング

| 要求ID | 設計セクション |
|---|---|
| FR-001（範囲チェック） | 1.4.1 `check_param_ranges` 関数 |
| FR-002（警告表示） | 1.4.2 表示処理 |
| FR-003（既存動作の非破壊） | 1.4.3 呼び出し位置と例外方針 |

## 1.2 システム構成

- 変更ファイル: `phase0/estimate_camera_params.py` のみ
  - 閾値定数の追加（ファイル冒頭の既存定数群の直後）
  - `check_param_ranges()` 関数の追加
  - `run_estimation()`（K未知モード側）に表示セクションの追加
- 新規ファイル: `tests/test_feat025_param_range_check.py`
- `phase0/common.py`、K既知モード（`_run_extrinsic_estimation`）は変更しない。

依存関係: `check_param_ranges` は他モジュールに依存しない純粋関数（標準ライブラリのみ使用）。

## 1.3 技術スタック

- Python 3.10、uv（ルート `pyproject.toml`）
- 新規ライブラリなし。テストは既存の pytest を使用。

## 1.4 各機能の詳細設計

### 1.4.1 `check_param_ranges` 関数（FR-001）

#### 閾値定数

`MIN_POINTS_*` 定数群の直後に以下を追加する:

```python
# パラメータ範囲チェックの閾値（feat-025）
F_SCALE_MIN = 0.3        # fx, fy の下限: 0.3 × max(width, height)
F_SCALE_MAX = 3.0        # fx, fy の上限: 3.0 × max(width, height)
FX_FY_RATIO_MIN = 0.9    # fx/fy 比の下限
FX_FY_RATIO_MAX = 1.1    # fx/fy 比の上限
CENTER_TOL_RATIO = 0.10  # cx, cy の許容ずれ: 画像幅/高さの10%
K_RADIAL_ABS_MAX = 1.0   # |k1|, |k2|, |k3| の上限
P_TANGENT_ABS_MAX = 0.01 # |p1|, |p2| の上限
```

#### シグネチャ

```python
def check_param_ranges(fx: float, fy: float, cx: float, cy: float,
                       k1: float, k2: float, p1: float, p2: float, k3: float,
                       img_width: int, img_height: int,
                       fix_center: bool, use_wide: bool, use_k3: bool,
                       zero_tangent: bool, estimate_distortion: bool) -> list[str]:
```

- 戻り値: 警告メッセージ（str）のリスト。全て範囲内なら空リスト。
- 引数の値域: fx〜k3 は `run_estimation` 内の `*_opt` 変数（float）。img_width, img_height は正の整数。

#### 処理ロジック（擬似コード）

```
warnings = []
S = max(img_width, img_height)

# 1. fx, fy の絶対範囲
if not (F_SCALE_MIN * S <= fx <= F_SCALE_MAX * S):
    warnings に「fx = {fx:.2f} が正常範囲 [{F_SCALE_MIN*S:.1f}, {F_SCALE_MAX*S:.1f}] を外れています」を追加
fy も同様

# 2. fx/fy 比（fy == 0 の場合は比を計算せず「fy = 0.00 が...」の fy 警告のみで足りるため、
#    ZeroDivisionError 回避として fy != 0 のときのみ判定する）
if fy != 0 and not (FX_FY_RATIO_MIN <= fx / fy <= FX_FY_RATIO_MAX):
    warnings に「fx/fy = {fx/fy:.4f} が正常範囲 [0.9, 1.1] を外れています」を追加

# 3. cx, cy（fix_center が False のときのみ）
if not fix_center:
    if not (img_width/2 - CENTER_TOL_RATIO*img_width <= cx <= img_width/2 + CENTER_TOL_RATIO*img_width):
        warnings に cx の警告を追加
    cy も同様（img_height 基準）

# 4. 歪み係数（estimate_distortion が True のときのみ）
if estimate_distortion:
    if not use_wide:                      # 有理モデルでは放射係数チェックをスキップ
        if abs(k1) > K_RADIAL_ABS_MAX: warnings に k1 の警告を追加
        if abs(k2) > K_RADIAL_ABS_MAX: warnings に k2 の警告を追加
        if use_k3 and abs(k3) > K_RADIAL_ABS_MAX: warnings に k3 の警告を追加
    if not zero_tangent:                  # p1, p2 は --wide でもチェックする
        if abs(p1) > P_TANGENT_ABS_MAX: warnings に p1 の警告を追加
        if abs(p2) > P_TANGENT_ABS_MAX: warnings に p2 の警告を追加

return warnings
```

- 警告メッセージのフォーマット（全項目共通）:
  `"{名前} = {値:.6f} が正常範囲 [{下限}, {上限}] を外れています"`
  （fx, fy, cx, cy は `:.2f`、fx/fy は `:.4f`、歪み係数は `:.6f`。歪み係数の範囲表示は `[-1.0, 1.0]` / `[-0.01, 0.01]` の形式）
- 境界値ちょうど（例: fx = 0.3×S、|p1| = 0.01）は範囲内とする（比較は `<=` / `>=`）。

#### 境界条件

- fy = 0: 項目1・2で fy の絶対範囲警告が出る。比の判定はスキップ（上記のとおり）。
- 歪み係数0固定（点数不足で `estimate_distortion == False`）: 歪み係数チェックは全てスキップ。内部パラメータのチェックは行う。
- `--zero-tangent` 時: p1, p2 は0固定なのでチェックしない（固定値への警告は無意味）。k1, k2（および `--k3` 時の k3）はチェックする。
- `--wide` 時: k1, k2, k3（および k4〜k6）はチェックしない。p1, p2 はチェックする。

#### エラーハンドリング

- 本関数は比較演算のみで例外を送出しない（唯一の除算は fy != 0 ガード済み）。
- 呼び出し側でも try/except は設けない（FR-003 の「例外で出力を妨げない」は上記設計により満たされる。握りつぶし用の except を置くと不具合の検出が遅れるため置かない — 設計判断、1.9 参照）。

### 1.4.2 表示処理（FR-002）

`run_estimation` の「[各点の再投影誤差]」表示ループの直後、「Calib_scene.toml 形式」ヘッダーの直前に以下を追加する:

```python
# パラメータ範囲チェック（feat-025）
warnings = check_param_ranges(
    fx_opt, fy_opt, cx_opt, cy_opt,
    k1_opt, k2_opt, p1_opt, p2_opt, k3_opt,
    img_width, img_height,
    fix_center, use_wide, use_k3, zero_tangent, estimate_distortion
)
print("\n[パラメータ範囲チェック]")
if warnings:
    for w in warnings:
        print(f"⚠ 警告: {w}")
    if use_wide:
        hint = ("--fix-center の使用、または標準モデル（--wide なし）への切り替えを"
                "検討してください。")
    else:
        hint = "--zero-tangent / --fix-center の使用を検討してください。"
    print(f"→ 点対応の誤り・点配置の偏り・パラメータ相殺（過学習）の可能性があります。{hint}")
else:
    print("✓ すべてのパラメータが正常範囲内です")
```

（コード例は意図の伝達が目的。変数名は実コードの `*_opt` に合わせる。）

- 対処ヒント行は警告が1件以上のとき末尾に1回だけ表示する。ヒント文は `use_wide` で切り替える（`--wide` と `--zero-tangent` は CLI で併用不可のため、`--wide` 時に `--zero-tangent` を勧めない。FR-002）。
- k4〜k6（`--wide` 時のみ存在）は関数に渡さない（チェック対象外のため）。

### 1.4.3 呼び出し位置と例外方針（FR-003）

- 呼び出しは K未知モードの `run_estimation` 内のみ。K既知モード（`_run_extrinsic_estimation`）には追加しない。
- 表示のみで、戻り値・終了コード・TOML/CSV 出力は一切変更しない。
- 全モード（オプションなし / `--fix-center` / `--k3` / `--wide` / `--zero-tangent` および CLI で併用可能な組み合わせ、歪み0固定ケース）で呼び出される。`--wide` と `--zero-tangent` の併用は既存 CLI がエラーにするため対象外（`use_wide=True, zero_tangent=True` の組み合わせは実装・テストの対象に含めない）。各モードでの適用条件は 1.4.1 の引数フラグで表現される。

## 1.5 状態遷移

該当なし（ステートレスな CLI 処理）。

## 1.6 ファイル・ディレクトリ設計

- 入出力ファイルの追加・変更なし。標準出力にセクションが1つ増えるのみ。
- テスト結果は `tests/results/feat-025_test_result.txt` に保存する（pytest -v の出力）。

## 1.7 インターフェース定義

- 追加公開関数: `check_param_ranges`（シグネチャは 1.4.1）。デフォルト引数なし（呼び出し側で全引数を明示する）。
- 既存関数のシグネチャ変更なし。
- 依存方向: `run_estimation` → `check_param_ranges`（一方向。循環なし）。

## 1.8 ログ・デバッグ設計

- 既存スクリプトと同様、logging は使わず print による日本語出力。
- 警告行の接頭辞は「⚠ 警告: 」、正常時は「✓ 」（既存の評価表示の記号体系に合わせる）。

## 1.9 設計判断の記録（簡易ADR）

| 判断 | 採用案 | 却下案と理由 |
|---|---|---|
| チェックの配置 | `estimate_camera_params.py` 内の純粋関数 | `common.py` へ切り出し: 他スクリプトから使う予定がなく、閾値定数と表示が本スクリプト固有のため却下 |
| 警告時の挙動 | 表示のみ・終了コード不変 | 終了コード変更: 既存のバッチ運用を壊す恐れがあるため却下（FR-003） |
| `--wide` の放射係数 | チェックしない | 個別閾値でのチェック: 有理モデルは分子分母の係数が相殺し合い、大きな値でも正常であり得るため誤警告になる |
| K既知モードのチェック | 対象外 | TOML 読み込み値のチェック: 推定値でなく入力値であり、本案件の課題（推定異常の見逃し）と別問題。必要なら別案件 |
| 例外の握りつぶし | try/except を置かない | 全体 try/except: 関数が例外を出さない設計であり、握りつぶしはテストでの不具合検出を妨げるため却下 |
| 閾値の由来 | p1,p2: 接線歪みは物理的に微小（典型 0.001 以下）のため 0.01。k1〜k3: 通常レンズで \|k\|>1 は過学習の典型のため 1.0。fx/fy 比: 正方画素前提で ±10%。cx,cy: 主点は画像中心近傍が物理的前提のため ±10%。f: 超広角〜望遠をカバーする 0.3〜3.0×max(w,h) | — |

## 1.10 テスト設計

`tests/test_feat025_param_range_check.py` を新規作成する。`check_param_ranges` を import してユニットテストする（`sys.path` に `phase0/` を追加する方式は既存テスト `test_feat023_zero_tangent.py` に倣う）。

テストケース（画像サイズは 960x540、S=960 を基準とする）:

1. 全パラメータ正常（fx=fy=1000, cx=480, cy=270, 歪み全て0）→ 空リスト
2. p1=0.5 → p1 の警告を含む（本案件の動機となったケース）
3. fx=200（< 0.3×960=288）→ fx の警告
4. fx=1200, fy=1000（比1.2）→ fx/fy 比の警告
5. cx=600（480+0.10×960=576 超）→ cx の警告
6. k1=1.5 → k1 の警告
7. 境界値: fx=288.0（=0.3×S）, p1=0.01, k1=1.0 → 警告なし（境界は範囲内）
8. fix_center=True で cx=9999 → cx の警告なし
9. use_wide=True で k1=5.0, p1=0.5 → k1 の警告なし、p1 の警告あり
10. zero_tangent=True で p1=0.0（固定値）→ p1 の警告なし。k1=1.5 → k1 の警告あり
11. estimate_distortion=False で k1=1.5（形式上渡す）→ 歪み警告なし
12. use_k3=False で k3=5.0 → k3 の警告なし / use_k3=True で k3=5.0 → 警告あり
13. fy=0 → fy の絶対範囲警告あり、例外が発生しない
14. 複数項目同時異常（p1=0.5, k1=2.0）→ 警告が2件

### 統合テスト（FR-002 / FR-003）

上記ユニットテストに加え、`run_estimation` を直接呼び出して標準出力を捕捉する統合テストを同ファイルに追加する。実データに依存しないよう、合成データを `tmp_path` に生成して使う:

- **合成データの生成**: 既知のカメラパラメータ（fx=fy=800, cx=480, cy=270, 歪みなし, 画像サイズ 960x540）と 20 点の非平面 3D 点（グリッド状 + Z 変化）を定め、`cv2.projectPoints` で 2D 点を生成する。`kijunten_locations.csv`・`points_2d.csv`・`config.yaml` を `tmp_path` に書き出し、`run_estimation(config_path, use_k3=False, use_wide=False, fix_center=False, zero_tangent=False)` を呼ぶ。
- **正常系テスト**（capsys で stdout を捕捉）:
  1. `[パラメータ範囲チェック]` セクションが出力に含まれる
  2. 「✓ すべてのパラメータが正常範囲内です」が含まれる（合成データは歪みなしの正確な投影のため、推定値は正常範囲に収束する）
  3. 「Calib_scene.toml 形式」「camera_params.csv 形式」の両セクションが含まれる（FR-003）
  4. 戻り値が 0（FR-003）
- **警告系テスト**: `check_param_ranges` を monkeypatch して固定の警告リスト（例: `["p1 = 0.500000 が正常範囲 [-0.01, 0.01] を外れています"]`）を返させ、同じ合成データで `run_estimation` を実行し以下を確認する（推定結果を実際に異常値へ誘導せず、表示経路を確実に通すため）:
  1. 「⚠ 警告: p1 = 0.500000 ...」行が含まれる
  2. `--wide` なしのヒント行（「--zero-tangent / --fix-center の使用を検討してください。」を含む行）が1回だけ含まれる
  3. 「Calib_scene.toml 形式」「camera_params.csv 形式」の両セクションが含まれ、戻り値が 0（FR-003: 警告があっても出力・戻り値は不変）

`--wide` 時のヒント文の切り替えは、`use_wide=True` を含む monkeypatch 統合テスト1件（「--fix-center の使用、または標準モデル（--wide なし）への切り替え」を含む行の確認。合成データは 20 点で MIN_POINTS_DIST8=20 を満たす）で確認する。
