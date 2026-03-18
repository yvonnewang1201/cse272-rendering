/// Curve strands implementation for fur/hair rendering using Embree's native curve primitives.
/// Key advantage: Embree provides the curve tangent direction, which is essential for hair BCSDF.

uint32_t register_embree_op::operator()(const CurveStrands &curves) const {
    // Select the appropriate Embree curve type
    RTCGeometryType geom_type;
    switch (curves.curve_type) {
        case CurveType::Linear:
            geom_type = RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE;
            break;
        case CurveType::Bezier:
            geom_type = RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE;
            break;
        case CurveType::BSpline:
            geom_type = RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE;
            break;
        case CurveType::CatmullRom:
            geom_type = RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE;
            break;
        default:
            geom_type = RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE;
    }

    RTCGeometry rtc_geom = rtcNewGeometry(device, geom_type);
    uint32_t geomID = rtcAttachGeometry(scene, rtc_geom);

    // Set control points buffer (x, y, z, radius) as FLOAT4
    float *control_points = (float*)rtcSetNewGeometryBuffer(
        rtc_geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4,
        sizeof(float) * 4, curves.control_points.size());

    for (size_t i = 0; i < curves.control_points.size(); i++) {
        const Vector4 &cp = curves.control_points[i];
        control_points[i * 4 + 0] = (float)cp[0];  // x
        control_points[i * 4 + 1] = (float)cp[1];  // y
        control_points[i * 4 + 2] = (float)cp[2];  // z
        control_points[i * 4 + 3] = (float)cp[3];  // radius
    }

    // Set index buffer
    unsigned int *indices = (unsigned int*)rtcSetNewGeometryBuffer(
        rtc_geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,
        sizeof(unsigned int), curves.indices.size());

    for (size_t i = 0; i < curves.indices.size(); i++) {
        indices[i] = (unsigned int)curves.indices[i];
    }

    rtcCommitGeometry(rtc_geom);
    rtcReleaseGeometry(rtc_geom);
    return geomID;
}

PointAndNormal sample_point_on_shape_op::operator()(const CurveStrands &curves) const {
    // Sample a curve segment based on area distribution
    int curve_id = sample(curves.curve_sampler, w);
    assert(curve_id >= 0 && curve_id < (int)curves.indices.size());

    int start_idx = curves.indices[curve_id];
    const Vector4 &p0 = curves.control_points[start_idx];
    const Vector4 &p1 = curves.control_points[start_idx + 1];

    // Sample along the curve segment
    Real t = uv[0];
    Vector3 pos = Vector3{p0[0], p0[1], p0[2]} * (1 - t) +
                  Vector3{p1[0], p1[1], p1[2]} * t;
    Real radius = p0[3] * (1 - t) + p1[3] * t;

    // Sample around the cylinder
    Real phi = 2 * c_PI * uv[1];

    // Tangent direction along the curve
    Vector3 tangent = normalize(Vector3{p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]});

    // Create orthonormal basis
    Vector3 bitangent, normal_dir;
    std::tie(bitangent, normal_dir) = coordinate_system(tangent);

    // Sample point on the cylinder surface
    Vector3 local_normal = cos(phi) * bitangent + sin(phi) * normal_dir;
    Vector3 point = pos + radius * local_normal;

    return PointAndNormal{point, local_normal};
}

Real surface_area_op::operator()(const CurveStrands &curves) const {
    return curves.total_area;
}

Real pdf_point_on_shape_op::operator()(const CurveStrands &curves) const {
    return 1 / surface_area_op{}(curves);
}

void init_sampling_dist_op::operator()(CurveStrands &curves) const {
    // Compute approximate surface area for each curve segment (as cylinders)
    std::vector<Real> segment_areas(curves.indices.size(), Real(0));
    Real total_area = 0;

    for (size_t i = 0; i < curves.indices.size(); i++) {
        int start_idx = curves.indices[i];
        const Vector4 &p0 = curves.control_points[start_idx];
        const Vector4 &p1 = curves.control_points[start_idx + 1];

        // Length of segment
        Real length = sqrt(
            (p1[0] - p0[0]) * (p1[0] - p0[0]) +
            (p1[1] - p0[1]) * (p1[1] - p0[1]) +
            (p1[2] - p0[2]) * (p1[2] - p0[2])
        );

        // Average radius
        Real avg_radius = (p0[3] + p1[3]) / 2;

        // Approximate surface area as a cylinder
        segment_areas[i] = 2 * c_PI * avg_radius * length;
        total_area += segment_areas[i];
    }

    curves.curve_sampler = make_table_dist_1d(segment_areas);
    curves.total_area = total_area;
}

ShadingInfo compute_shading_info_op::operator()(const CurveStrands &curves) const {
    // For curves, Embree provides:
    // - vertex.primitive_id: curve segment index
    // - vertex.st[0]: u parameter along curve (0 to 1)
    // - vertex.st[1]: v parameter around curve (0 to 1)
    // - vertex.geometric_normal: For round curves, this is the curve tangent!

    int segment_id = vertex.primitive_id;
    assert(segment_id >= 0 && segment_id < (int)curves.indices.size());

    int start_idx = curves.indices[segment_id];
    const Vector4 &p0 = curves.control_points[start_idx];
    const Vector4 &p1 = curves.control_points[start_idx + 1];

    // =========================================================================
    // v32 FIX: SEPARATE UV FIELDS
    // =========================================================================
    // - uv: Curve parameterization from Embree (u along curve, v around cylinder)
    //       This is what PathVertex.uv will become after compute_shading_info
    // - surface_uv: The ROOT UV from mesh texture map (for spotted fur pattern)
    //       This MUST be stored separately because uv gets overwritten!
    // =========================================================================
    Vector2 uv = vertex.st;  // Curve parameterization from Embree

    // =========================================================================
    // v44 FIX: ROBUST strand UV fetch with debug verification
    // =========================================================================
    // The UV pipeline was failing because segment_to_strand/root_uvs might
    // be empty or mismatched. This version adds robust fallbacks and debug.
    // =========================================================================
    Vector2 surface_uv{0.5, 0.5};  // Fallback if lookup fails
    bool has_surface_uv = false;

    // v44: Debug counters to verify UV pipeline (static for one-time output)
    static bool first_call = true;
    static int lookup_success_count = 0;
    static int lookup_failure_count = 0;

    // Force the lookup - no early exit conditions
    int strand_id = -1;

    // v44: Verify arrays are populated
    bool segment_to_strand_valid = !curves.segment_to_strand.empty() &&
                                   segment_id >= 0 &&
                                   segment_id < (int)curves.segment_to_strand.size();

    if (segment_to_strand_valid) {
        strand_id = curves.segment_to_strand[segment_id];
    }

    bool root_uvs_valid = !curves.root_uvs.empty() &&
                          strand_id >= 0 &&
                          strand_id < (int)curves.root_uvs.size();

    if (root_uvs_valid) {
        // HARD FETCH: Get the UV directly
        Vector2 raw_uv = curves.root_uvs[strand_id];

        // v34: Apply UV wrapping HERE (the parse_scene wrapping was dead)
        // Wrap to [0,1] range using floor
        surface_uv.x = raw_uv.x - floor(raw_uv.x);
        surface_uv.y = raw_uv.y - floor(raw_uv.y);
        has_surface_uv = true;
        lookup_success_count++;
    } else {
        lookup_failure_count++;
    }

    // v44: One-time debug output to stderr
    if (first_call) {
        first_call = false;
        std::cerr << "[UV Debug] First curve hit: segment_id=" << segment_id
                  << ", indices.size=" << curves.indices.size()
                  << ", segment_to_strand.size=" << curves.segment_to_strand.size()
                  << ", root_uvs.size=" << curves.root_uvs.size()
                  << ", segment_to_strand_valid=" << segment_to_strand_valid
                  << ", root_uvs_valid=" << root_uvs_valid;
        if (has_surface_uv) {
            std::cerr << ", surface_uv=(" << surface_uv.x << "," << surface_uv.y << ")";
        }
        std::cerr << std::endl;
    }

    // =========================================================================
    // SURFACE NORMAL: Direct lookup by control point index (v26 FIX)
    // Each control point has its own surface normal stored during parsing.
    // This is MORE RELIABLE than the strand_id lookup which was failing.
    // =========================================================================
    Vector3 skin_normal{0, 1, 0};  // Default to up
    bool has_skin_normal = false;

    if (!curves.surface_normals.empty() &&
        start_idx >= 0 && start_idx < (int)curves.surface_normals.size()) {
        skin_normal = curves.surface_normals[start_idx];
        has_skin_normal = true;
    }

    // Curve tangent direction (along the hair strand)
    // For hair BCSDF, this should be the "x" direction of the shading frame
    Vector3 tangent = normalize(Vector3{
        p1[0] - p0[0],
        p1[1] - p0[1],
        p1[2] - p0[2]
    });

    // The normal points radially outward from the curve
    // For round curves, vertex.geometric_normal is the radial direction
    Vector3 radial_normal = vertex.geometric_normal;

    // =========================================================================
    // v37 GEOMETRIC NORMAL SAFEGUARD
    // =========================================================================
    // At curve endpoints, tips, or grazing angles, the radial normal from
    // Embree can be degenerate (parallel to tangent or near-zero length).
    // This causes the orthogonalization to produce zero vectors, leading to
    // NaN/Inf in the shading frame and subsequent fireflies.
    //
    // Safeguard: Check for degenerate vectors and fall back to a safe default.
    // =========================================================================

    // Make sure radial_normal is perpendicular to tangent
    Vector3 orthogonalized = radial_normal - tangent * dot(radial_normal, tangent);
    Real orthog_len_sq = dot(orthogonalized, orthogonalized);

    // v37: Check for degenerate normal (parallel to tangent or zero-length)
    static const Real MIN_NORMAL_LENGTH_SQ = Real(1e-8);
    if (orthog_len_sq < MIN_NORMAL_LENGTH_SQ) {
        // Degenerate case: construct a fallback normal perpendicular to tangent
        // Use coordinate_system to generate a reliable perpendicular vector
        Vector3 fallback_bitangent, fallback_normal;
        std::tie(fallback_bitangent, fallback_normal) = coordinate_system(tangent);
        radial_normal = fallback_normal;
    } else {
        radial_normal = orthogonalized / sqrt(orthog_len_sq);  // normalize
    }

    // =========================================================================
    // SLICK FILM NORMAL BLENDING (v25)
    // When wetness > 0, blend the shading normal from cylinder toward skin.
    // This creates the appearance of a continuous glossy oil film by making
    // adjacent hairs reflect light as if they were a single smooth surface.
    // =========================================================================
    Vector3 shading_normal = radial_normal;
    if (curves.wetness > Real(0) && has_skin_normal) {
        // Blend from cylinder normal toward skin normal
        // wetness=0: pure cylinder (dry fur)
        // wetness=1: pure skin normal (slick oil film)
        Vector3 blended = radial_normal * (Real(1) - curves.wetness) +
                          skin_normal * curves.wetness;
        Real blended_len_sq = dot(blended, blended);
        if (blended_len_sq > MIN_NORMAL_LENGTH_SQ) {
            shading_normal = blended / sqrt(blended_len_sq);
            // Re-orthogonalize: ensure shading_normal is perpendicular to tangent
            Vector3 orthog_shading = shading_normal - tangent * dot(shading_normal, tangent);
            Real orthog_shading_len_sq = dot(orthog_shading, orthog_shading);
            if (orthog_shading_len_sq > MIN_NORMAL_LENGTH_SQ) {
                shading_normal = orthog_shading / sqrt(orthog_shading_len_sq);
            }
            // else: keep the non-orthogonalized shading_normal (edge case)
        }
        // else: keep radial_normal as shading_normal (degenerate blend)
    }

    // Bitangent completes the orthonormal basis
    Vector3 bitangent = cross(shading_normal, tangent);
    // v37: Ensure bitangent is normalized (cross product of unit vectors is unit)
    Real bitangent_len_sq = dot(bitangent, bitangent);
    if (bitangent_len_sq > MIN_NORMAL_LENGTH_SQ) {
        bitangent = bitangent / sqrt(bitangent_len_sq);
    } else {
        // Degenerate case - use coordinate system fallback
        Vector3 fb_bitangent, fb_normal;
        std::tie(fb_bitangent, fb_normal) = coordinate_system(tangent);
        bitangent = fb_bitangent;
        shading_normal = fb_normal;
    }

    // For Hair BCSDF, we need the tangent to be the primary direction
    // Standard hair convention: x = tangent (along hair), n = shading normal
    Frame shading_frame(tangent, bitangent, shading_normal);

    // Compute approximate dp/du for inv_uv_size
    Real segment_length = sqrt(
        (p1[0] - p0[0]) * (p1[0] - p0[0]) +
        (p1[1] - p0[1]) * (p1[1] - p0[1]) +
        (p1[2] - p0[2]) * (p1[2] - p0[2])
    );
    Real avg_radius = (p0[3] + p1[3]) / 2;
    Real inv_uv_size = max(segment_length, Real(2 * c_PI * avg_radius));

    // Mean curvature for a cylinder of radius r is 1/(2r)
    Real mean_curvature = Real(1) / (2 * avg_radius);

    // v32: Return surface_uv for texture lookup (separate from curve parameterization)
    return ShadingInfo{uv, shading_frame, mean_curvature, inv_uv_size,
                       skin_normal, has_skin_normal,
                       surface_uv, has_surface_uv};
}
