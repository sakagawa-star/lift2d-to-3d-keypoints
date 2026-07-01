# FPS風カメラ 頭部追従ハンドラ（参考資料）

3Dキーポイント（アーマチュア）の頭部に仮想カメラを追従させ、顔の一次視線方向を向かせるBlenderハンドラ。3D-GS室内での一人称視点レンダリング用。

## 前提となるシーン構成

- アーマチュア名: `session001_f145749_world300`（各キーポイントがボーン）
- 使用ボーン: `LEye`, `REye`, `LEar`, `REar`, `Nose`, `Head`, `Neck`
- Empty `Cam_Anchor`: 両目中点に固定（Armatureコンストレイント、ターゲット `LEye`/`REye` 各ウェイト1.0）
- カメラ `Cam_FPS`: `Cam_Anchor` の子（親子付け、ローカル回転ゼロ）

位置はコンストレイントが担い、**向きだけをこのハンドラが計算**する。

## 座標系の定義（頭部姿勢）

医学・光学分野のFrankfurt平面ベースの頭部座標系に準拠。頭部姿勢を視線方向とみなす（眼球運動は含まない）。

3軸の定義:

- 前方 `f`: 一次視線方向。耳中点→鼻ベクトル `p` を、両目軸に直交化したもの。`f = normalize(p - r0 * dot(p, r0))`
- 上 `u`: `f × r0`。頭頂方向（`Head - Neck`）との内積で符号を確定（推測ではなく事実基準）。
- 右 `r`: `u × b`（`b = -f`）。右手系を数学的に保証するために外積で導出。

## 手系とto_euler()の注意（重要）

Blenderの `Matrix.to_euler()` は**行列式+1の直交行列（正しい回転行列）**を前提とする。行列式が-1（鏡像）の行列を渡すと、符号を正しく分解できず往復変換で反転する。

このため、3軸を手で列に並べるだけでは不十分。カメラのローカル基底は右手系（右 × 上 = 後方、X × Y = +Z）なので、`r = u × b` で右軸を導出し、`Matrix((r, u, b)).transposed()` の行列式が必ず+1になるよう構成する。

検証項目:

- `rot.determinant() == 1.0`
- `rot.is_orthogonal == True`
- `to_euler().to_matrix()` の往復で列が保存される

## カメラ規約

Blenderのカメラはローカル **-Z が前方**、**+Y が上**、**+X が右**。よって回転行列の列は `(右, 上, 後方) = (r, u, b)`（`b = -f`）。

## 最終ハンドラ

```python
import bpy
from bpy.app.handlers import persistent
from mathutils import Matrix

ARM_NAME = "session001_f145749_world300"
ANCHOR_NAME = "Cam_Anchor"

@persistent
def update_face_orientation(scene, depsgraph=None):
    arm = bpy.data.objects.get(ARM_NAME)
    anchor = bpy.data.objects.get(ANCHOR_NAME)
    if arm is None or anchor is None:
        return

    mw = arm.matrix_world
    LEye = mw @ arm.pose.bones["LEye"].head
    REye = mw @ arm.pose.bones["REye"].head
    LEar = mw @ arm.pose.bones["LEar"].head
    REar = mw @ arm.pose.bones["REar"].head
    Nose = mw @ arm.pose.bones["Nose"].head
    Head = mw @ arm.pose.bones["Head"].head
    Neck = mw @ arm.pose.bones["Neck"].head
    ear_mid = (LEar + REar) * 0.5

    p = (Nose - ear_mid).normalized()             # 前方の手がかり（耳中点→鼻）
    r0 = (LEye - REye).normalized()               # 両目軸（直交化の基準）
    f = (p - r0 * p.dot(r0)).normalized()         # 前方（両目軸に直交＝一次視線）
    u = f.cross(r0).normalized()                  # 上（暫定）
    head_up = (Head - Neck).normalized()          # 頭頂方向（符号確定の基準）
    if u.dot(head_up) < 0:
        u = -u
    b = -f                                        # カメラ ローカル+Z = 後方
    r = u.cross(b).normalized()                   # r = u×b で右手系を保証（det=+1）

    rot = Matrix((r, u, b)).transposed()          # 列 = (右, 上, 後方)
    anchor.rotation_euler = rot.to_euler()

bpy.app.handlers.frame_change_post.clear()
bpy.app.handlers.frame_change_post.append(update_face_orientation)

update_face_orientation(bpy.context.scene)
```

## 動作と制約

- `frame_change_post` に登録。`scene.frame_set()` やタイムライン再生のたびに発火し、カメラが頭部に追従。
- ハンドラは .blend に保存されない。ファイルを開き直すと解除されるため、対話GUIでは再実行が必要。
- `blender -b`（バックグラウンド別プロセス）ではこのハンドラは起動しない。バッチ処理では、同じ計算をバッチスクリプト側に組み込む必要がある。
