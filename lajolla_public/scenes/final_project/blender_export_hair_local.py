"""
Blender Hair Export Script - LOCAL COORDINATES (matches OBJ export)

IMPORTANT: This exports hair in LOCAL/OBJECT space, NOT world space.
This ensures the fur aligns with the OBJ mesh when both have the same
transform applied in the renderer.

Usage:
1. Select the object with the particle hair system
2. Run this script in Blender's Scripting workspace
3. Use the same transform for BOTH obj and curves in lajolla
"""

import bpy
import os
from mathutils import Vector, Matrix

def export_particle_hair_local(obj, filepath,
                                steps_per_strand=6,
                                radius_root=0.0031,
                                radius_tip=0.0001):
    """
    Export particle hair in LOCAL space (same as OBJ export).
    """

    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)

    # Get inverse of world matrix to convert back to local space
    world_to_local = obj_eval.matrix_world.inverted()

    particle_systems = [ps for ps in obj_eval.particle_systems
                       if ps.settings.type == 'HAIR']

    if not particle_systems:
        print(f"No hair particle systems found on {obj.name}")
        return 0

    total_strands = 0

    with open(filepath, 'w') as f:
        f.write(f"# Lajolla Hair Export - LOCAL COORDINATES\n")
        f.write(f"# Object: {obj.name}\n")
        f.write(f"# IMPORTANT: Use same transform for OBJ and curves in lajolla\n")
        f.write(f"# Format: x y z radius\n\n")

        for ps in particle_systems:
            particles = ps.particles

            for p_idx, particle in enumerate(particles):
                if not particle.is_exist:
                    continue

                hair_keys = particle.hair_keys
                num_keys = len(hair_keys)

                if num_keys < 2:
                    continue

                for i in range(steps_per_strand):
                    t = i / (steps_per_strand - 1)

                    key_float = t * (num_keys - 1)
                    key_idx = min(int(key_float), num_keys - 2)
                    key_frac = key_float - key_idx

                    # Get positions from hair keys - these are in WORLD space
                    pos1 = hair_keys[key_idx].co_object(obj_eval,
                                                        ps.settings.hair_step,
                                                        particle)
                    pos2 = hair_keys[key_idx + 1].co_object(obj_eval,
                                                            ps.settings.hair_step,
                                                            particle)

                    # Interpolate in world space
                    pos_world = pos1.lerp(pos2, key_frac)

                    # Convert to world space first (co_object gives object-relative)
                    pos_world = obj_eval.matrix_world @ pos_world

                    # Then convert back to LOCAL space (matching OBJ coordinates)
                    pos_local = world_to_local @ pos_world

                    # Taper radius from root to tip
                    radius = radius_root * (1 - t) + radius_tip * t

                    f.write(f"{pos_local.x:.6f} {pos_local.y:.6f} {pos_local.z:.6f} {radius:.6f}\n")

                f.write("\n")
                total_strands += 1

    return total_strands


def main():
    obj = bpy.context.active_object

    if obj is None:
        print("ERROR: No object selected!")
        return

    blend_dir = os.path.dirname(bpy.data.filepath) if bpy.data.filepath else "/tmp"
    output_path = os.path.join(blend_dir, f"{obj.name}_hair_local.txt")

    print(f"Exporting hair from '{obj.name}' to '{output_path}'...")
    print("NOTE: Exporting in LOCAL coordinates (same as OBJ)")

    num_strands = export_particle_hair_local(
        obj, output_path,
        steps_per_strand=6,
        radius_root=0.0031,
        radius_tip=0.0001
    )

    print(f"Done! Exported {num_strands} hair strands")
    print(f"\nIn lajolla, use SAME transform for both:")
    print(f'''
<shape type="obj">
    <string name="filename" value="Seal.obj"/>
    <transform name="toWorld">
        <rotate y="1" angle="45"/>
        <scale value="1.8"/>
    </transform>
</shape>

<shape type="curves">
    <string name="filename" value="{os.path.basename(output_path)}"/>
    <transform name="toWorld">
        <rotate y="1" angle="45"/>
        <scale value="1.8"/>
    </transform>
</shape>
''')


if __name__ == "__main__":
    main()
