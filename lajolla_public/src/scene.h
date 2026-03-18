#pragma once

#include "lajolla.h"
#include "camera.h"
#include "light.h"
#include "material.h"
#include "medium.h"
#include "shape.h"
#include "volume.h"

#include <memory>
#include <vector>

enum class Integrator {
    Depth,
    ShadingNormal,
    MeanCurvature,
    RayDifferential, // visualize radius & spread
    MipmapLevel,
    Path,
    VolPath
};

struct RenderOptions {
    Integrator integrator = Integrator::Path;
    int samples_per_pixel = 4;
    int max_depth = -1;
    int rr_depth = 5;
    int vol_path_version = 0;
    int max_null_collisions = 1000;
};

/// Bounding sphere
struct BSphere {
    Real radius;
    Vector3 center;
};

/// A "Scene" contains the camera, materials, geometry (shapes), lights,
/// and also the rendering options such as number of samples per pixel or
/// the parameters of our renderer.
struct Scene {
    Scene() {}
    Scene(const RTCDevice &embree_device,
          const Camera &camera,
          const std::vector<Material> &materials,
          const std::vector<Shape> &shapes,
          const std::vector<Light> &lights,
          const std::vector<Medium> &media,
          int envmap_light_id, /* -1 if the scene has no envmap */
          const TexturePool &texture_pool,
          const RenderOptions &options,
          const std::string &output_filename);
    ~Scene();
    Scene(const Scene& t) = delete;
    Scene& operator=(const Scene& t) = delete;

    RTCDevice embree_device;
    RTCScene embree_scene;
    // We decide to maintain a copy of the scene here.
    // This allows us to manage the memory of the scene ourselves and decouple
    // from the scene parser, but it's obviously less efficient.
    Camera camera;
    // For now we use stl vectors to store scene content.
    // This wouldn't work if we want to extend this to run on GPUs.
    // If we want to port this to GPUs later, we need to maintain a thrust vector or something similar.
    const std::vector<Material> materials;
    const std::vector<Shape> shapes;
    const std::vector<Light> lights;
    const std::vector<Medium> media;
    int envmap_light_id;
    const TexturePool texture_pool;

    // Bounding sphere of the scene.
    BSphere bounds;
    
    RenderOptions options;
    std::string output_filename;

    // For sampling lights
    TableDist1D light_dist;
};

/// Sample a light source from the scene given a random number u \in [0, 1]
int sample_light(const Scene &scene, Real u);

/// The probability mass function of the sampling procedure above.
Real light_pmf(const Scene &scene, int light_id);

inline bool has_envmap(const Scene &scene) {
    return scene.envmap_light_id != -1;
}

inline const Light &get_envmap(const Scene &scene) {
    assert(scene.envmap_light_id != -1);
    return scene.lights[scene.envmap_light_id];
}

inline Real get_shadow_epsilon(const Scene &scene) {
    return min(scene.bounds.radius * Real(1e-5), Real(0.01));
}

inline Real get_intersection_epsilon(const Scene &scene) {
    return min(scene.bounds.radius * Real(1e-5), Real(0.01));
}

// =============================================================================
// v37 CURVE RAY EPSILON FIX
// =============================================================================
// Problem: Curve radii are ~0.0001 or smaller. Standard epsilon (~1e-5) causes
// immediate self-intersection with micro-cylinders at grazing angles.
//
// Solution: When spawning rays from curve hits, use a MUCH larger epsilon
// proportional to the curve radius. This escapes the "self-intersection trap"
// where rays continuously re-hit the same or adjacent strands.
// =============================================================================

/// Check if a shape is a curve/hair primitive
inline bool is_curve_shape(const Shape &shape) {
    return std::holds_alternative<CurveStrands>(shape);
}

/// Get the average radius of a curve at a given primitive (segment) ID
inline Real get_curve_radius_at_hit(const Shape &shape, int primitive_id) {
    if (!std::holds_alternative<CurveStrands>(shape)) {
        return Real(0);
    }
    const CurveStrands &curves = std::get<CurveStrands>(shape);
    if (primitive_id < 0 || primitive_id >= (int)curves.indices.size()) {
        return Real(0.0001);  // Fallback to typical fur radius
    }
    int start_idx = curves.indices[primitive_id];
    if (start_idx < 0 || start_idx + 1 >= (int)curves.control_points.size()) {
        return Real(0.0001);
    }
    // Average of start and end radius for this segment
    Real r0 = curves.control_points[start_idx][3];
    Real r1 = curves.control_points[start_idx + 1][3];
    return (r0 + r1) / Real(2);
}

/// Get dynamic intersection epsilon based on hit geometry
/// For curves: 20x the curve radius to escape micro-cylinder self-intersection
/// For other geometry: standard scene epsilon
inline Real get_intersection_epsilon_for_vertex(const Scene &scene, const PathVertex &vertex) {
    if (vertex.shape_id < 0 || vertex.shape_id >= (int)scene.shapes.size()) {
        return get_intersection_epsilon(scene);
    }
    const Shape &shape = scene.shapes[vertex.shape_id];
    if (is_curve_shape(shape)) {
        // v37: Dynamic epsilon based on curve radius
        // Use 20x the curve radius to robustly escape self-intersection
        Real curve_radius = get_curve_radius_at_hit(shape, vertex.primitive_id);
        Real curve_epsilon = curve_radius * Real(20);
        // Also ensure minimum epsilon based on scene scale
        return max(curve_epsilon, get_intersection_epsilon(scene));
    }
    return get_intersection_epsilon(scene);
}

/// Get dynamic shadow epsilon based on hit geometry
/// For curves: use same scaling as intersection epsilon
inline Real get_shadow_epsilon_for_vertex(const Scene &scene, const PathVertex &vertex) {
    if (vertex.shape_id < 0 || vertex.shape_id >= (int)scene.shapes.size()) {
        return get_shadow_epsilon(scene);
    }
    const Shape &shape = scene.shapes[vertex.shape_id];
    if (is_curve_shape(shape)) {
        Real curve_radius = get_curve_radius_at_hit(shape, vertex.primitive_id);
        Real curve_epsilon = curve_radius * Real(20);
        return max(curve_epsilon, get_shadow_epsilon(scene));
    }
    return get_shadow_epsilon(scene);
}
