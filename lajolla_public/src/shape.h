#pragma once

#include "lajolla.h"
#include "frame.h"
#include "table_dist.h"
#include "vector.h"
#include <embree4/rtcore.h>
#include <variant>
#include <vector>

struct PointAndNormal;
struct PathVertex;

struct ShadingInfo {
    Vector2 uv; // UV coordinates for texture mapping
    Frame shading_frame; // the coordinate basis for shading
    Real mean_curvature; // 0.5 * (dN/du + dN/dv)
    // Stores min(length(dp/du), length(dp/dv)), for ray differentials.
    Real inv_uv_size;
    // Baked skin/macro-surface normal for curves (from export script)
    // Used by OilCoatedHairBCSDF for smooth clearcoat specular
    Vector3 skin_normal = Vector3{0, 1, 0};  // Default to up
    bool has_skin_normal = false;  // True if skin_normal was baked
    // v32: Surface UV for curves - the ROOT UV from mesh texture map
    // This is NEVER overwritten by curve parameterization
    Vector2 surface_uv = Vector2{0.5, 0.5};
    bool has_surface_uv = false;
};

/// A Shape is a geometric entity that describes a surface. E.g., a sphere, a triangle mesh, a NURBS, etc.
/// For each shape, we also store an integer "material ID" that points to a material, and an integer
/// "area light ID" that points to a light source if the shape is an area light. area_lightID is set to -1
/// if the shape is not an area light.
struct ShapeBase {
    int material_id = -1;
    int area_light_id = -1;

    int interior_medium_id = -1;
    int exterior_medium_id = -1;
};

struct Sphere : public ShapeBase {
    Vector3 position;
    Real radius;
};

struct TriangleMesh : public ShapeBase {
    /// TODO: make these portable to GPUs
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    /// Below are used only when the mesh is associated with an area light
    Real total_area;
    /// For sampling a triangle based on its area
    TableDist1D triangle_sampler;
};

/// Curve strands for fur/hair rendering using Embree's native curve primitives.
/// Each curve is a sequence of control points with radii.
/// Embree provides proper tangent direction along the curve for hair BCSDF.
enum class CurveType {
    Linear,         // Linear segments (RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE)
    Bezier,         // Cubic Bezier curves (RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE)
    BSpline,        // B-Spline curves (RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE)
    CatmullRom     // Catmull-Rom splines (RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE)
};

struct CurveStrands : public ShapeBase {
    /// Control points with radius stored as (x, y, z, radius)
    std::vector<Vector4> control_points;
    /// Indices into control_points for each curve segment
    /// For linear curves: each index is a segment start (2 points)
    /// For cubic curves: each index is a segment start (4 points)
    std::vector<int> indices;
    /// UV coordinates at the root of each strand (for texture lookup)
    /// One UV per strand, indexed by strand_id
    std::vector<Vector2> root_uvs;
    /// Baked skin/macro-surface normals at the root of each strand
    /// Used for wet fur clearcoat specular (OilCoatedHairBCSDF)
    /// One normal per strand, indexed by strand_id
    std::vector<Vector3> skin_normals;
    /// Surface normals per control point (same size as control_points)
    /// Each control point stores the skin normal for normal blending
    /// This enables direct lookup by vertex index during intersection
    std::vector<Vector3> surface_normals;
    /// Maps segment index to strand index (for UV lookup and skin normal)
    std::vector<int> segment_to_strand;
    /// Number of points per curve segment
    int points_per_segment = 2;  // 2 for linear, 4 for cubic
    /// Type of curve interpolation
    CurveType curve_type = CurveType::Linear;
    /// Total surface area (approximate, for sampling)
    Real total_area = 0;
    /// For sampling curves based on their area
    TableDist1D curve_sampler;
    /// If true, these curves don't cast shadows on other shadow-invisible curves.
    /// Used for wet fur to avoid micro-self-shadowing between dense strands.
    bool shadow_invisible = false;
    /// Wetness factor for normal blending (0=dry cylinder, 1=skin normal)
    /// Used to create "slick oil film" appearance by blending shading normal
    /// from hair cylinder toward baked skin normal.
    Real wetness = Real(0);
};

// To add more shapes, first create a struct for the shape, add it to the variant below,
// then implement all the relevant functions below.
using Shape = std::variant<Sphere, TriangleMesh, CurveStrands>;

/// Add the shape to an Embree scene.
uint32_t register_embree(const Shape &shape, const RTCDevice &device, const RTCScene &scene);

/// Sample a point on the surface given a reference point.
/// uv & w are uniform random numbers.
PointAndNormal sample_point_on_shape(const Shape &shape,
                                     const Vector3 &ref_point,
                                     const Vector2 &uv,
                                     Real w);

/// Probability density of the operation above
Real pdf_point_on_shape(const Shape &shape,
                        const PointAndNormal &point_on_shape,
                        const Vector3 &ref_point);

/// Useful for sampling.
Real surface_area(const Shape &shape);

/// Some shapes require storing sampling data structures inside. This function initialize them.
void init_sampling_dist(Shape &shape);

/// Embree doesn't calculate some shading information for us. We have to do it ourselves.
ShadingInfo compute_shading_info(const Shape &shape, const PathVertex &vertex);

inline void set_material_id(Shape &shape, int material_id) {
    std::visit([&](auto &s) { s.material_id = material_id; }, shape);
}
inline void set_area_light_id(Shape &shape, int area_light_id) {
    std::visit([&](auto &s) { s.area_light_id = area_light_id; }, shape);
}
inline void set_interior_medium_id(Shape &shape, int interior_medium_id) {
    std::visit([&](auto &s) { s.interior_medium_id = interior_medium_id; }, shape);
}
inline void set_exterior_medium_id(Shape &shape, int exterior_medium_id) {
    std::visit([&](auto &s) { s.exterior_medium_id = exterior_medium_id; }, shape);
}
inline int get_material_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.material_id; }, shape);
}
inline int get_area_light_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.area_light_id; }, shape);
}
inline int get_interior_medium_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.interior_medium_id; }, shape);
}
inline int get_exterior_medium_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.exterior_medium_id; }, shape);
}
inline bool is_light(const Shape &shape) {
    return get_area_light_id(shape) >= 0;
}

/// Check if a shape is a CurveStrands
inline bool is_curve(const Shape &shape) {
    return std::holds_alternative<CurveStrands>(shape);
}

/// Check if a shape is a CurveStrands with shadow_invisible=true
inline bool is_shadow_invisible_curve(const Shape &shape) {
    if (const CurveStrands *curves = std::get_if<CurveStrands>(&shape)) {
        return curves->shadow_invisible;
    }
    return false;
}
