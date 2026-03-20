import bpy
import numpy as np
import json

scene = bpy.context.scene
cam = bpy.data.objects['FPSCamera']

# 3DGSオブジェクトのKIRIモディファイヤを一時無効化
kiri_objects = [obj for obj in scene.objects if 'KIRI_3DGS_Render_GN' in obj.modifiers]
for obj in kiri_objects:
    obj.modifiers['KIRI_3DGS_Render_GN'].show_viewport = False

camera_data = []
for frame in range(scene.frame_start, scene.frame_end + 1):
    scene.frame_set(frame)
    render = scene.render
    focal_px = (cam.data.lens / cam.data.sensor_width) * render.resolution_x
    camera_data.append({
        'frame': frame,
        'c2w': np.array(cam.matrix_world).tolist(),
        'fx': focal_px,
        'fy': focal_px,
        'cx': render.resolution_x / 2,
        'cy': render.resolution_y / 2,
        'width': render.resolution_x,
        'height': render.resolution_y,
    })

# モディファイヤを元に戻す
for obj in kiri_objects:
    obj.modifiers['KIRI_3DGS_Render_GN'].show_viewport = True

with open('/home/sakagawa/git/lift2d-to-3d-keypoints/phase4/data/FPS-camera_poses.json', 'w') as f:
    json.dump(camera_data, f)
