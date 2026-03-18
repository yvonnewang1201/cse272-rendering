# Realistic Spotted Seal Fur Rendering (Dry and Wet)

**CSE 272 – Advanced Rendering** | UC San Diego | Yvonne Wang

---

## Project Overview

This project implements a physically-based renderer that simulates two distinct appearances of a spotted harbor seal's fur coat — **dry** and **wet** — by extending the [lajolla](https://github.com/BachiLi/lajolla_public) path tracer with custom hair/fur BSDFs and curve geometry.

| Dry Seal | Wet Seal |
|:---:|:---:|
| <img src="results/seal_dry.png" width="480"/> | <img src="results/seal_wet.png" width="480"/> |

### Dry → Wet Transition

![Dry to Wet Animation](results/dry_to_wet.apng)

---

## Key Techniques

### 1. Marschner Hair BCSDF (Dry Fur)
Implemented the full Marschner hair scattering model with three lobes:
- **R** (surface reflection) — produces the bright highlight at the cuticle
- **TT** (direct transmission) — responsible for the forward-scattering glow
- **TRT** (internal reflection + transmission) — back-scattering glint

To simulate a dense pelt (not sparse single strands), a **diffuse fallback** term is blended in to approximate multiple scattering between neighboring strands, preserving the silver-gray base color and dark spot pattern without violating energy conservation.

### 2. Oil-Coated Hair BCSDF (Wet Fur)
When a seal enters water, a thin oil film coats each hair shaft. The key physics:
- **IOR matching**: oil film (η ≈ 1.50) closely matches hair keratin (η ≈ 1.55), reducing the effective Fresnel reflectance to ~0.07% — the fur becomes optically "invisible" as a diffuse scatterer
- **Dual-lobe GGX specular**: a sharp lobe (α = 0.005) for point-light highlights and a wide lobe (α = 0.15) for broad environment reflections capture the characteristic wet sheen
- Importance sampling draws exclusively from the wide lobe to eliminate PDF-mismatch fireflies

### 3. Curve Geometry via Embree
Fur strands are represented as Embree `RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE` primitives exported from Blender. The wet scene applies **geometric flattening** (compressing strand height along the skin normal) and **normal blending** (interpolating the cylinder normal toward the skin normal) to simulate clumping.

### 4. UV Pipeline for Spotted Texture
Each curve strand stores the **surface UV** of its root point on the skin mesh. The BSDFs look up the spot texture (`seal_spot.jpg`) at this UV coordinate, so dark spots appear consistently across both geometry and shading.

### 5. Firefly Elimination
The combination of micro-cylinder geometry (r ~ 0.0001) and HDR environment maps produces extreme sample variance. A nuclear sample clamp at the render accumulation level eliminates all fireflies without darkening the image.

---

## Repository Structure

```
lajolla_public/
├── src/
│   ├── materials/
│   │   ├── hair_bcsdf.inl          # Marschner Hair BCSDF (dry fur)
│   │   └── oil_coated_hair.inl     # Oil-Coated Hair BCSDF (wet fur)
│   ├── shapes/
│   │   └── curve_strands.inl       # Fur strand curve geometry + UV pipeline
│   ├── render.cpp                  # Nuclear sample clamp
│   ├── path_tracing.h              # MC weight / indirect radiance clamps
│   ├── intersection.cpp/h          # Curve intersection epsilon
│   ├── material.cpp/h              # BSDF dispatch for hair/oilcoatedhair
│   ├── shape.cpp/h                 # Curve shape parsing + wetness params
│   ├── scene.h                     # Scene epsilon helpers
│   └── parsers/parse_scene.cpp     # XML parameter parsing
└── scenes/final_project/
    ├── seal_dry_final.xml          # Final dry seal scene
    ├── seal_wet_final.xml          # Final wet seal scene
    ├── Seal_Body.obj               # Seal body mesh
    ├── Seal_Eyes.obj               # Seal eyes mesh
    ├── ground_plane.obj            # Ground plane
    ├── fur_base.txt                # Fur strand data — NOT included (see note below)
    └── seal_spot.jpg               # Spot texture (skin UV map)

results/
    ├── seal_dry.png                # Final dry render
    ├── seal_wet.png                # Final wet render
    └── dry_to_wet.apng             # Animated dry→wet transition (APNG)
```

---

## How to Run

### Prerequisites
- CMake ≥ 3.17, C++17 compiler (clang or gcc)
- Intel Embree 3 (included in `lajolla_public/embree/`)

### Build
```bash
cd lajolla_public
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Render

**Dry Seal:**
```bash
cd lajolla_public/scenes/final_project
../../build/lajolla -o seal_dry.exr ./seal_dry_final.xml
```

**Wet Seal:**
```bash
cd lajolla_public/scenes/final_project
../../build/lajolla -o seal_wet.exr ./seal_wet_final.xml
```

> **Note:** Pass the XML with a `./` prefix (e.g. `./seal_dry_final.xml`) so the scene parser correctly resolves relative asset paths.

### Convert EXR → PNG
```bash
ffmpeg -i seal_dry.exr -vf "exposure=0.8,tonemap=clip,format=rgb24" \
    ../../homework/final_project/seal_dry.png
ffmpeg -i seal_wet.exr -vf "exposure=0.8,tonemap=clip,format=rgb24" \
    ../../homework/final_project/seal_wet.png
```

### Generate Transition Animation
```bash
python3 homework/final_project/make_dry_wet_gif.py
# Output: homework/final_project/dry_to_wet.apng
```

---

## Note: Fur Strand Data Not Included

`fur_base.txt` (~300 MB) exceeds GitHub's 100 MB file size limit and is therefore not tracked in this repository. It contains the raw fur strand geometry (positions, radii, and root UV coordinates for ~10 million curve segments) exported from Blender.

To regenerate it, open `lajolla_public/scenes/final_project/Untitled1.blend` in Blender and run the export script:

```bash
blender Untitled1.blend --background --python \
    lajolla_public/scenes/final_project/blender_export_hair_curves.py
```

The script writes `fur_base.txt` into the `scenes/final_project/` directory. Place it there before rendering either scene.

---

## References

- Marschner et al., *Light Scattering from Human Hair Fibers* (SIGGRAPH 2003)
- d'Eon et al., *An Energy-Conserving Hair Reflectance Model* (EGSR 2011)
- Chiang et al., *A Practical and Controllable Hair and Fur Model* (EGSR 2016)
- Pharr, Jakob, Humphreys, *Physically Based Rendering* (4th ed.)
