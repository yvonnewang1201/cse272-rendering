CSE 272 Homework 2 - Volumetric Path Tracing
============================================

Required Implementations
------------------------
All six volumetric path tracers have been implemented in:
  src/vol_path_tracing.h

1. vol_path_tracing_1: Single absorption homogeneous volume (line 124)
2. vol_path_tracing_2: Single scattering with NEE (line 170)
3. vol_path_tracing_3: Multiple scattering homogeneous (line 340)
4. vol_path_tracing_4: Multiple scattering with NEE (line 490)
5. vol_path_tracing_5: Full volumetric path tracer with MIS (line 680)
6. vol_path_tracing_6: Heterogeneous volumes with delta tracking (line 900)

Required Renders
----------------
- volpath_test1.exr: vol_path_tracing_1 test
- volpath_test2.exr: vol_path_tracing_2 test
- volpath_test3.exr: vol_path_tracing_3 test
- volpath_test4.exr: vol_path_tracing_4 test
- volpath_test4_2.exr: vol_path_tracing_4 additional test
- volpath_test5.exr: vol_path_tracing_5 test
- volpath_test5_2.exr: vol_path_tracing_5 additional test
- volpath_test6.exr: vol_path_tracing_6 test
- vol_cbox.exr: Cornell box with participating media
- vol_cbox_teapot.exr: Cornell box with teapot in fog
- hetvol.exr: Heterogeneous volume (bunny smoke)
- hetvol_colored.exr: Colored heterogeneous volume

Bonus 1: Equiangular Sampling (35%)
-----------------------------------
Implementation: src/vol_path_tracing.h (lines 1-117)

Equiangular sampling is implemented as described in:
"Importance Sampling Techniques for Path Tracing in Participating Media"
by Christopher Kulla and Marcos Fajardo, EGSR 2012

Key functions:
- sample_equiangular(): Samples distance using equiangular distribution
- pdf_equiangular(): Computes PDF for equiangular sampling

The sampling strategy uses one-sample MIS between transmittance sampling
and equiangular sampling with 50/50 probability, using the balance heuristic.

Applied to: vol_path_tracing_2, vol_path_tracing_4, vol_path_tracing_5

Comparison renders:
- volpath_test2_with_equiangular.exr: With equiangular MIS (current implementation)
- volpath_test2_no_equiangular.exr: Transmittance sampling only

The equiangular sampling reduces variance especially when there is a point
or small area light source inside participating media, as it importance
samples the geometric term 1/|p - light|^2 along the ray.

Bonus 2: Custom Scene (20%)
---------------------------
Scene file: scenes/homework2_test/jellyfish_sea.xml

Description: An underwater jellyfish scene rendered with volumetric path
tracing. The scene features:
- Ocean medium with absorption and scattering
- Translucent jellyfish body with internal scattering medium
- Bioluminescent glow from the jellyfish core
- Sunlight filtering from above

Rendered output: jellyfish_bonus.exr

The scene demonstrates the volumetric rendering capabilities including:
- Multiple scattering in participating media
- Transmittance through heterogeneous boundaries
- Area light sources in volumetric environments

Build Instructions
------------------
cd lajolla_public
mkdir build && cd build
cmake ..
make -j

Running the renderer:
./lajolla <scene.xml>
