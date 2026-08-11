"""実験 e1 用: stdin の各行に到着時刻（epoch秒、小数3桁）を前置して出力するフィルタ。

criteria.md §4.3 で lock されたタイムスタンプ付与の実体。使用例:

    PYTHONUNBUFFERED=1 <レンダリングコマンド> 2>&1 | python3 ts_filter.py > run_p1.log

標準ライブラリのみ使用（uv 環境不要、システム python3 で動く）。
"""
import sys
import time


def main() -> int:
    for line in sys.stdin:
        sys.stdout.write(f"{time.time():.3f} {line}")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
