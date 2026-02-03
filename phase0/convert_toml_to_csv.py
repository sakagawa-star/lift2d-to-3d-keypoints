"""
Calib_scene.toml を camera_params.csv に変換するスクリプト

使い方:
    python convert_toml_to_csv.py Calib_scene.toml camera_params.csv
"""

import argparse
import csv
import re
from pathlib import Path


def parse_toml_simple(toml_path: str) -> dict:
    """
    簡易TOMLパーサー（標準ライブラリのみ使用）
    Calib_scene.toml の形式に特化
    """
    cameras = {}
    current_section = None
    
    with open(toml_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            
            # 空行・コメント行をスキップ
            if not line or line.startswith('#'):
                continue
            
            # セクション開始 [section_name]
            if line.startswith('[') and line.endswith(']'):
                current_section = line[1:-1]
                if current_section != 'metadata':
                    cameras[current_section] = {}
                continue
            
            # キー = 値 の解析
            if '=' in line and current_section and current_section != 'metadata':
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                
                # 配列の解析 [ 1.0, 2.0, 3.0 ] or [[ 1.0, 2.0 ], [ 3.0, 4.0 ]]
                if value.startswith('['):
                    # 数値を抽出
                    numbers = re.findall(r'-?\d+\.?\d*(?:e[+-]?\d+)?', value, re.IGNORECASE)
                    cameras[current_section][key] = [float(n) for n in numbers]
                # 文字列の解析 "value"
                elif value.startswith('"') and value.endswith('"'):
                    cameras[current_section][key] = value[1:-1]
                # ブール値
                elif value.lower() in ('true', 'false'):
                    cameras[current_section][key] = value.lower() == 'true'
                # 数値
                else:
                    try:
                        cameras[current_section][key] = float(value)
                    except ValueError:
                        cameras[current_section][key] = value
    
    return cameras


def convert_to_csv(cameras: dict, output_path: str):
    """
    パースしたカメラデータをCSVに変換
    """
    header = [
        'camera_name', 'width', 'height',
        'fx', 'fy', 'cx', 'cy',
        'k1', 'k2', 'p1', 'p2',
        'r1', 'r2', 'r3',
        't1', 't2', 't3'
    ]
    
    rows = []
    for section_name, params in cameras.items():
        # サイズ
        size = params.get('size', [1920.0, 1080.0])
        width, height = int(size[0]), int(size[1])
        
        # 内部パラメータ（matrix は 3x3 を flatten したもの）
        matrix = params.get('matrix', [])
        if len(matrix) >= 9:
            fx = matrix[0]  # [0][0]
            fy = matrix[4]  # [1][1]
            cx = matrix[2]  # [0][2]
            cy = matrix[5]  # [1][2]
        else:
            fx, fy, cx, cy = 0, 0, 0, 0
        
        # 歪み係数
        dist = params.get('distortions', [0, 0, 0, 0])
        while len(dist) < 4:
            dist.append(0)
        k1, k2, p1, p2 = dist[0], dist[1], dist[2], dist[3]
        
        # 外部パラメータ
        rotation = params.get('rotation', [0, 0, 0])
        translation = params.get('translation', [0, 0, 0])
        r1, r2, r3 = rotation[0], rotation[1], rotation[2]
        t1, t2, t3 = translation[0], translation[1], translation[2]
        
        # カメラ名（セクション名から抽出、例: int_cam01_img -> cam01）
        camera_name = section_name
        # "int_" プレフィックスと "_img" サフィックスを除去
        if camera_name.startswith('int_'):
            camera_name = camera_name[4:]
        if camera_name.endswith('_img'):
            camera_name = camera_name[:-4]
        
        row = [
            camera_name, width, height,
            fx, fy, cx, cy,
            k1, k2, p1, p2,
            r1, r2, r3,
            t1, t2, t3
        ]
        rows.append(row)
    
    # CSV書き出し
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    
    print(f"変換完了: {output_path}")
    print(f"カメラ数: {len(rows)}")
    for row in rows:
        print(f"  - {row[0]}")


def main():
    parser = argparse.ArgumentParser(
        description='Calib_scene.toml を camera_params.csv に変換'
    )
    parser.add_argument('input', help='入力ファイル (Calib_scene.toml)')
    parser.add_argument('output', help='出力ファイル (camera_params.csv)')
    
    args = parser.parse_args()
    
    # 入力ファイル存在確認
    if not Path(args.input).exists():
        print(f"エラー: 入力ファイルが見つかりません: {args.input}")
        return 1
    
    # 変換実行
    cameras = parse_toml_simple(args.input)
    convert_to_csv(cameras, args.output)
    
    return 0


if __name__ == '__main__':
    exit(main())
