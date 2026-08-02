**指摘**

- **中** [requirements.md:37](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:37), [requirements.md:42](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:42) が、[design.md:105](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:105) の「カメラ単位不整合は当該カメラ失敗で続行」とまだ完全には揃っていません。画像サイズの受け入れ基準1は修正済みですが、FR-001概要は「不整合は処理開始前に停止」、受け入れ基準2は6点対応不成立を「エラー終了」と読めます。  
  **修正提案**: FR-001を「全体停止する入力不備」と「カメラ単位失敗で続行する不整合」に分け、6点対応不成立・座標範囲外・非有限値の扱いを design §4.1 と同じ文言に固定する。

- **中** [requirements.md:112](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/requirements.md:112) と [design.md:98](/home/sakagawa/git/lift2d-to-3d-keypoints/docs/issues/feat-026-render-match-extrinsic-refinement/design.md:98) で、`--cameras` 省略時の対象カメラ集合が異なります。requirements は「6点対応がある全カメラ」、design は「2D CSV に当該カメラ名の行がある全カメラ」です。部分的な2D行だけあるカメラを、未対象として飛ばすのか、対象にしてカメラ失敗として記録するのかが分岐します。  
  **修正提案**: design 準拠にするなら requirements 側を「2D点 CSV に当該カメラ名の行がある全カメラ。6点対応不成立は FR-001 のカメラ単位失敗」に変更する。

**確認済み**

FR-001受け入れ基準1の画像サイズ不一致は design §4.1 に揃っています。`pyproject.toml` の `testpaths = ["tests"]` も有効で、`UV_CACHE_DIR=/tmp/uv-cache uv run pytest --collect-only -q` では `tests` 配下の271件のみ収集され、`docs/issues/.../stage2_smoke_test.py` は収集対象から外れていました。