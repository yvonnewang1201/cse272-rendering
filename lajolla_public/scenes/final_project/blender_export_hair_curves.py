import bpy
import os
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import barycentric_transform

"""
Spotted Seal Fur Export Script
Exports two types of fur:
1. dry_fur.txt - Guard Hair + Underfur (fluffy appearance)
2. wet_fur.txt - Clumped fur (wet/oily appearance)
"""

# Output paths
output_dir = "/Users/wangkaiwei/Documents/ucsd/cse272/project/lajolla_public/scenes/final_project/"

# 取得選取的海豹
obj = bpy.context.active_object

if not obj or len(obj.particle_systems) == 0:
    print("錯誤：請選取帶有粒子系統的海豹！")
else:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)
    mat_world = obj_eval.matrix_world

    # 取得 mesh 資料
    mesh = obj_eval.data
    
    # ==========================================
    # 核心修復：使用 BMesh 與 BVH Tree 幾何計算 UV
    # ==========================================
    print("正在建立 UV 幾何查詢結構...")
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.triangulate(bm, faces=bm.faces[:]) # 轉為三角面以利精確計算
    bm.faces.ensure_lookup_table()
    bvh = BVHTree.FromBMesh(bm)
    
    uv_layer = bm.loops.layers.uv.active

    def get_particle_uv(local_pos):
        """利用物理座標去尋找網格上的精確 UV，避開 buggy API"""
        if not uv_layer:
            return (0.5, 0.5)
            
        location, normal, face_idx, dist = bvh.find_nearest(local_pos)
        if face_idx is None:
            return (0.5, 0.5)
            
        face = bm.faces[face_idx]
        if len(face.verts) == 3:
            v1, v2, v3 = face.verts[0].co, face.verts[1].co, face.verts[2].co
            uv1, uv2, uv3 = face.loops[0][uv_layer].uv, face.loops[1][uv_layer].uv, face.loops[2][uv_layer].uv
            
            # 關鍵修正：將 2D 的 UV 轉成 3D 向量 (u, v, 0.0) 騙過 Blender API
            uv1_3d = Vector((uv1.x, uv1.y, 0.0))
            uv2_3d = Vector((uv2.x, uv2.y, 0.0))
            uv3_3d = Vector((uv3.x, uv3.y, 0.0))
            
            # 使用重心座標內插算出精確 UV
            uv = barycentric_transform(location, v1, v2, v3, uv1_3d, uv2_3d, uv3_3d)
            return (uv.x, uv.y)
        return (0.5, 0.5)

    # ========================================
    # DRY FUR: Guard Hair + Underfur
    # ========================================
    print("\n=== 匯出乾燥毛皮 (dry_fur.txt) ===")
    dry_fur_path = os.path.join(output_dir, "dry_fur.txt")
    total_dry = 0

    with open(dry_fur_path, 'w') as f:
        f.write("# Spotted Seal DRY FUR\n")
        f.write("# Guard Hair (長毛) + Underfur (底毛)\n")
        f.write("# Format: STRAND root_u root_v\n")
        f.write("#         x y z radius\n\n")

    # Export Guard Hair (較長、較粗)
    for psys in obj_eval.particle_systems:
        if "Guard" in psys.name or "guard" in psys.name:
            print(f"處理 Guard Hair: {psys.name}")
            strand_count = 0
            with open(dry_fur_path, 'a') as f:
                f.write(f"# === Guard Hair: {psys.name} ===\n")
                for p in psys.particles:
                    num_keys = len(p.hair_keys)
                    if num_keys < 2: continue
                    
                    # 傳入毛髮根部的座標，讓 BVH 去找 UV
                    root_uv = get_particle_uv(p.hair_keys[0].co)
                    f.write(f"STRAND {root_uv[0]:.6f} {root_uv[1]:.6f}\n")

                    for i in range(num_keys):
                        local_pos = p.hair_keys[i].co
                        world_pos = mat_world @ local_pos
                        
                        out_x, out_y, out_z = world_pos.x, world_pos.z, -world_pos.y
                        
                        t = i / (num_keys - 1)
                        radius = 0.00009 * (1.0 - t) + 0.00007 * t
                        f.write(f"{out_x:.6f} {out_y:.6f} {out_z:.6f} {radius:.6f}\n")
                    f.write("\n")
                    strand_count += 1
            total_dry += strand_count

    # Export Underfur (較短、較細)
    for psys in obj_eval.particle_systems:
        if "Under" in psys.name or "under" in psys.name:
            print(f"處理 Underfur: {psys.name}")
            strand_count = 0
            with open(dry_fur_path, 'a') as f:
                f.write(f"# === Underfur: {psys.name} ===\n")
                for p in psys.particles:
                    num_keys = len(p.hair_keys)
                    if num_keys < 2: continue

                    root_uv = get_particle_uv(p.hair_keys[0].co)
                    f.write(f"STRAND {root_uv[0]:.6f} {root_uv[1]:.6f}\n")

                    for i in range(num_keys):
                        local_pos = p.hair_keys[i].co
                        world_pos = mat_world @ local_pos
                        out_x, out_y, out_z = world_pos.x, world_pos.z, -world_pos.y
                        
                        t = i / (num_keys - 1)
                        radius = 0.00005 * (1.0 - t) + 0.00003 * t
                        f.write(f"{out_x:.6f} {out_y:.6f} {out_z:.6f} {radius:.6f}\n")
                    f.write("\n")
                    strand_count += 1
            total_dry += strand_count

    print(f"DRY FUR 總計: {total_dry} strands")

    # ========================================
    # WET FUR: Only Guard Hair
    # ========================================
    print("\n=== 匯出濕潤毛皮 (wet_fur.txt) ===")
    wet_fur_path = os.path.join(output_dir, "wet_fur.txt")
    total_wet = 0

    with open(wet_fur_path, 'w') as f:
        f.write("# Spotted Seal WET FUR\n")
        f.write("# Clumped guard hair only\n")
        f.write("# Format: STRAND root_u root_v\n")
        f.write("#         x y z radius\n\n")

    for psys in obj_eval.particle_systems:
        if "Guard" in psys.name or "guard" in psys.name:
            print(f"處理 Wet Guard Hair: {psys.name}")
            strand_count = 0
            with open(wet_fur_path, 'a') as f:
                f.write(f"# === Wet Guard Hair: {psys.name} ===\n")
                for p in psys.particles:
                    num_keys = len(p.hair_keys)
                    if num_keys < 2: continue

                    root_uv = get_particle_uv(p.hair_keys[0].co)
                    f.write(f"STRAND {root_uv[0]:.6f} {root_uv[1]:.6f}\n")

                    for i in range(num_keys):
                        local_pos = p.hair_keys[i].co
                        world_pos = mat_world @ local_pos
                        out_x, out_y, out_z = world_pos.x, world_pos.z, -world_pos.y
                        
                        t = i / (num_keys - 1)
                        radius = 0.00009 * (1.0 - t) + 0.00008 * t
                        f.write(f"{out_x:.6f} {out_y:.6f} {out_z:.6f} {radius:.6f}\n")
                    f.write("\n")
                    strand_count += 1
            total_wet += strand_count

    print(f"WET FUR 總計: {total_wet} strands")
    
    # 釋放記憶體
    bm.free()
    
    print("\n" + "="*50)
    print("所有毛髮匯出完成！兩份檔案已成功生成！")
    print("="*50)