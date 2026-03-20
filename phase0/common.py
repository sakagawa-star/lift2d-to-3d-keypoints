"""共通関数モジュール"""

import csv

import numpy as np


def load_yaml_simple(yaml_path: str) -> dict:
    """簡易YAMLパーサー"""
    config = {}
    with open(yaml_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' in line:
                key, value = line.split(':', 1)
                config[key.strip()] = value.strip()
    return config


def load_points_3d(csv_path: str) -> dict:
    """3D座標を読み込む"""
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row['ObjectName']
            points[name] = [float(row['X']), float(row['Y']), float(row['Z'])]
    return points


def load_points_2d(csv_path: str, camera_name: str) -> dict:
    """2D座標を読み込む"""
    points = {}
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['camera_name'] == camera_name:
                name = row['ObjectName']
                points[name] = [float(row['X']), float(row['Y'])]
    return points


def match_points(points_3d: dict, points_2d: dict):
    """3Dと2Dの点をマッチング"""
    matched_3d = []
    matched_2d = []
    names = []

    for name in points_3d:
        if name in points_2d:
            matched_3d.append(points_3d[name])
            matched_2d.append(points_2d[name])
            names.append(name)

    return (
        np.array(matched_3d, dtype=np.float64),
        np.array(matched_2d, dtype=np.float64),
        names
    )
