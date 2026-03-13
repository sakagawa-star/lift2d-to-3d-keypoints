import bpy
import csv
import os
import re

# 出力先CSVファイルのパス（Blenderファイルと同じディレクトリに保存）
output_filename = "kijunten_locations.csv"
blend_dir = bpy.path.abspath("//")
if not blend_dir:
    # Blenderファイルが未保存の場合はホームディレクトリに保存
    blend_dir = os.path.expanduser("~")
output_path = os.path.join(blend_dir, output_filename)

# "基準_" + 数字 にマッチするパターン
pattern = re.compile(r'^基準_\d+$')

# "基準点" コレクション内のオブジェクトを取得
collection_name = "基準点"
collection = bpy.data.collections.get(collection_name)

if collection is None:
    print(f"[ERROR] コレクション '{collection_name}' が見つかりません。")
else:
    matched_objects = [
        obj for obj in collection.all_objects
        if pattern.match(obj.name)
    ]

    if not matched_objects:
        print(f"[WARNING] '{collection_name}' 内に 'パターン 基準_NNNN' に一致するオブジェクトが見つかりませんでした。")
    else:
        with open(output_path, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            # ヘッダ行
            writer.writerow(["ObjectName", "X", "Y", "Z"])
            # データ行
            for obj in matched_objects:
                loc = obj.location
                writer.writerow([obj.name, loc.x, loc.y, loc.z])
                print(f"  {obj.name}: ({loc.x}, {loc.y}, {loc.z})")

        print(f"\n[OK] {len(matched_objects)} 件のオブジェクトを '{output_path}' に保存しました。")
