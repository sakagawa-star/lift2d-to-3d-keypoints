再レビュー結果: 致命的な指摘はありません。

**高**
なし。

**中**
なし。

**低**
なし。

§4.7 の修正は、実機で確認された根本原因である「ffmpeg の stdin が開いたまま `wait()` してワーカーが詰まる」問題に対して十分です。`stdin.close()` で EOF を送ってから `terminate()`、さらに `wait(timeout=10)` と `kill()` エスカレーションを入れる設計なので、親側の `_dispatch_loop` が worker の error/exitcode を検出できないまま待ち続ける経路は潰せています。

直列モードへの副作用も、設計上は許容範囲です。変更対象は `render_chunk()` の `KeyboardInterrupt` 捕捉節だけで、正常系の描画・エンコード・`durable_replace()` 経路は変更しない。Ctrl-C 時も `raise` で既存の終了コード130経路に戻るため、挙動の意味は維持されています。

requirements 側も、`render_chunk()` 無改変制約に KeyboardInterrupt 捕捉節限定の例外が追加されており、design §4.7 / ADR-8 / テスト14 / GPU実機確認3の再実施条件と整合しています。実装時はテスト14で `render` モジュールもフェイク注入する必要がありますが、これは設計の致命的欠陥ではなく実装時の注意点です。