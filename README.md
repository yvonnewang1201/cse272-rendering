# Advanced Rendering - CSE 272 Final Project

A physically-based renderer built on the lajolla framework, featuring advanced rendering techniques including volumetric path tracing, Disney BSDF materials, and importance sampling strategies.

## Features

### Implemented Techniques

**Volumetric Path Tracing**
- Homogeneous and heterogeneous participating media
- Multiple scattering with next event estimation (NEE)
- Delta tracking for heterogeneous volumes
- Henyey-Greenstein phase function

**Disney BSDF**
- Full Disney Principled BSDF implementation
- Anisotropic metal and glass materials
- Clearcoat layer with correct sampling
- Subsurface scattering approximation

**Importance Sampling**
- Equiangular sampling for point lights in participating media
- Multiple Importance Sampling (MIS) with balance heuristic
- Visible normal sampling (Heitz 2018) for anisotropic GGX

## Sample Renders

<!-- Add your rendered images here -->
| Scene | Description |
|-------|-------------|
| Coming soon | Volumetric Cornell Box |
| Coming soon | Heterogeneous Volume |
| Coming soon | Disney Materials |

## Build Instructions

```bash
cd lajolla_public
mkdir build && cd build
cmake ..
make -j
```

## Usage

```bash
./lajolla <scene.xml>
```

## Project Structure

```
├── lajolla_public/          # Main renderer source code
│   ├── src/
│   │   ├── materials/       # BSDF implementations
│   │   ├── vol_path_tracing.h  # Volumetric integrators
│   │   └── ...
│   └── scenes/              # Test scenes
├── homework/                # Homework implementations
│   ├── homework1/           # Disney BSDF
│   └── homework2/           # Volumetric path tracing
└── README.md
```

## References

- [Physically Based Rendering: From Theory to Implementation](https://pbrt.org/)
- [Extending the Disney BRDF to a BSDF with Integrated Subsurface Scattering](https://blog.selfshadow.com/publications/s2015-shading-course/)
- [Importance Sampling Techniques for Path Tracing in Participating Media](https://www.solidangle.com/research/egsr2012_volume.pdf)
- [Sampling the GGX Distribution of Visible Normals](https://jcgt.org/published/0007/04/01/)

## Author

Yvonne Wang - UCSD CSE 272 (Winter 2025)

## License

This project is for educational purposes as part of UCSD CSE 272.
