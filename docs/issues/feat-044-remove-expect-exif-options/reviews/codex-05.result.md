# codex-05（feat-044 範囲拡大後の書き直し版 初回レビュー）

- 日付: 2026-11-26
- 対象: requirements.md、design.md（範囲拡大後に全面的に書き直した版。関連: README.md、prepare_phone_photo.py、estimate_intrinsics_loop.py、parse_args_with_config、load_yaml_flat、テスト、feat-041/043 の文書）
- ストリーム: rev-lift2d-feat-044
- フェーズ: 反復（書き直し版の初回。`/new` で会話をクリアしてから依頼。依頼文に「致命的な点のみ指摘」の前提）
- 指摘数: 高 2 / 中 1 / 低 0
- トークン: total_tokens 478,085（input 474,638 うち cached 414,336、output 3,447、reasoning 704）
- rollout: ~/.codex/sessions/2026/11/26/rollout-2026-11-26T10-24-06-01a21573-e4b0-7da1-878e-a09cab28715d.jsonl

## 指摘

### 指摘1（高）書き出しが原本画像を破損し得る

設計 §4.1.2 は出力先だけを exists() で確認し、固定名 {out}.tmp に書き込む。この一時パスが既存のシンボリックリンクで原本を指していると、--overwrite なしでも原本が切り詰められる。--overwrite 付きで image と photo_info に同じパスを指定すると原本自体が置換される。feat-041 のデータ保全要件（NFR-004）に合わせ、入力と出力の衝突を拒否し、同じディレクトリに排他的に作成した一時ファイルで確定する設計にする。上書き禁止は確定時にも保証する。

### 指摘2（高）前処理が Exif の焦点距離を読み続けるため、写真情報ファイルで回避できない失敗が残る

FR-002 は前処理で FocalLengthIn35mmFilm を読まないと定めているが、設計 §4.2.2 のままでは前処理が呼ぶ read_exif が同タグを int に変換する。同タグに inf が入った JPEG を現行の read_exif に渡すと OverflowError になった。焦点距離タグの取得・変換を書き出し側だけに限定し、§6.2 No.10 のモック試験に加えて、Exif のない原本 JPEG と手書きの写真情報ファイルで前処理 CLI が完走することを確認する。

### 指摘3（中）写真情報ファイルの配置によって、共用 YAML で推定まで進めなくなる

photo_info: results/photo_info.yaml と out_dir: results を指定すると、書き出しが results を作成し、推定は既存ディレクトリとして終了コード 1 で停止する。tmp_dir でも同じ。手書きの場合も同じ制約がかかる。feat-043 の配置制約を photo_info にも適用し、out_dir・tmp_dir の配下に置けないことを FR-004 と利用手順に明記する。

## 対応

- 指摘1: FR-001 に入力と出力の衝突の拒否・確定時の上書き禁止・一時ファイルのデータ保全を追加。設計 §4.1.2 に `os.path.realpath` による衝突検査（`--overwrite` の有無に関係なく終了コード 1）と `os.path.lexists` による出力済み検査、§4.1.4（新設）に書き込み方式（`--overwrite` なしは `open(out, "x")` の排他的新規作成、ありは `tempfile.mkstemp(dir=out.parent)` + `os.replace`。固定名 `{out}.tmp` は使わない）。§6.2 No.13・14 を追加
- 指摘2: 前処理の `read_exif` から `FocalLengthIn35mmFilm` を削除（戻り値 3 項目）。書き出しに専用の `read_focal_35mm`（§4.1.3。`OverflowError` を含めて変換失敗は None）を新設し、§6.2 No.12 で変換を試験。§6.4 の 6 に、OpenCV で再エンコードした Exif の無い原本 JPEG と手書きの写真情報ファイルで前処理 CLI を完走させ、初期 TOML・2D 点 CSV を feat-043 の出力と cmp で比較する手順を追加
- 指摘3: FR-004 に `photo_info` を `out_dir`・`tmp_dir` の配下に置けない制約を追加。§8 の README 更新対象にも追記
