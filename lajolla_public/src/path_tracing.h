#pragma once

#include "scene.h"
#include "pcg.h"

// ============================================================================
// FIREFLY SUPPRESSION (v31): Path Space Filtering + Smart Clamping
// ============================================================================
// Two-pronged approach for eliminating fireflies on dense hair geometry:
//
// 1. PATH SPACE FILTERING (bounce-dependent roughness):
//    - Primary rays (bounce 0): Use material's actual roughness
//    - Indirect rays (bounce >= 1): Force min roughness 0.3
//    This mathematically widens the PDF on secondary bounces, preventing
//    singularities while keeping primary specular sharp.
//
// 2. SMART CLAMPING:
//    - NEE (direct): Soft clamp only for highly specular (roughness < 0.1)
//    - BSDF sampling: Tighter clamp for indirect bounces
//    - Throughput: Aggressive clamp to prevent exponential buildup
// ============================================================================

// Clamp limits (tuned for 1.3M overlapping hair strands)
static const Real MAX_THROUGHPUT_UPDATE = Real(8.0);         // Prevent throughput explosion
static const Real MAX_BSDF_CONTRIBUTION_PRIMARY = Real(25.0);  // First bounce
static const Real MAX_BSDF_CONTRIBUTION_INDIRECT = Real(10.0); // Secondary+ bounces
static const Real MAX_NEE_SPECULAR = Real(15.0);              // v35: Tightened from 50 to 15 for specular NEE

// v35 HARD FIX: Direct Monte Carlo weight clamp
// This clamps bsdf_val / pdf BEFORE any other processing
// The TT lobe PDF can approach infinity causing fireflies - this is the root fix
// v35: Tightened from 10.0 to 5.0 to eliminate residual fireflies
static const Real MAX_MC_WEIGHT = Real(5.0);

// Minimum roughness for path space filtering on indirect bounces
static const Real PATH_SPACE_MIN_ROUGHNESS = Real(0.3);

// =============================================================================
// v38 INDIRECT RADIANCE CLAMP - THE ACTUAL ROOT FIX
// =============================================================================
// Previous clamps failed because:
// - MC weight clamp: Useless when diffuse BSDF weight is already ≤ 1.0
// - G term clamp at 1e6: 1 million is still a massive firefly
// - Missing: The HDRI sun has radiance values of 10,000+ which multiply through
//
// The fix: Clamp incoming light radiance on INDIRECT bounces (bounce > 0).
// Primary rays (bounce 0) keep full dynamic range for specular highlights.
// =============================================================================
static const Real MAX_INDIRECT_RADIANCE = Real(10.0);  // Hard cap on light intensity

// v38: Clamp incoming radiance on indirect bounces
// This kills HDRI sun hotspots that cause fireflies in dense fur
inline Spectrum clamp_indirect_radiance(const Spectrum &L, int bounce) {
    if (bounce == 0) {
        return L;  // Primary rays: preserve full dynamic range
    }
    Spectrum result = L;
    for (int i = 0; i < 3; ++i) {
        result[i] = std::clamp(result[i], Real(0), MAX_INDIRECT_RADIANCE);
    }
    return result;
}

// v38: Reasonable geometry term clamp (was 1e6, now 100)
// 1e6 is absurdly high - a G of 100 is already extremely close intersection
static const Real MAX_GEOMETRY_TERM = Real(100.0);

// =============================================================================
// v39 FINAL SAMPLE CLAMP - THE INDUSTRY STANDARD "CATCH-ALL"
// =============================================================================
// Problem: Even with all previous clamps, multiplication chains can explode:
//   Radiance(10) × G(100) × Weight(5) = 5000 → still a firefly!
//
// Solution: Apply a FINAL per-sample clamp at the very end of the path tracer.
// This is how production renderers (Arnold, Cycles, etc.) guarantee no fireflies.
//
// The clamp value of 20.0 is chosen because:
// - Typical scene luminance range is 0-1 for diffuse, 0-10 for specular
// - Any sample > 20 is almost certainly a firefly
// - This preserves bright specular highlights while killing outliers
// =============================================================================
static const Real MAX_SAMPLE_VALUE = Real(20.0);

// v39: Final sample clamp - the ultimate gatekeeper
// Applied at the very end of path_tracing before return
inline Spectrum clamp_final_sample(const Spectrum &radiance) {
    Spectrum result = radiance;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(result[i]) || std::isinf(result[i])) {
            result[i] = Real(0);  // Kill NaN/Inf
        }
        result[i] = std::clamp(result[i], Real(0), MAX_SAMPLE_VALUE);
    }
    return result;
}

// =============================================================================
// v36 NaN/Inf TRAP: Detect mathematical singularities
// =============================================================================
// std::clamp CANNOT constrain NaN or Inf - they pass through unchanged!
// If fireflies survive a clamp of 5.0, they are NaN/Inf from divide-by-zero.
// This function detects and replaces them with BRIGHT MAGENTA for visualization.
// =============================================================================
static const Spectrum MAGENTA_NAN_MARKER = Spectrum{100.0, 0.0, 100.0};

inline bool is_valid_spectrum(const Spectrum &s) {
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(s[i]) || std::isinf(s[i])) {
            return false;
        }
    }
    return true;
}

inline Spectrum sanitize_spectrum(const Spectrum &s, bool use_magenta_marker = false) {
    if (!is_valid_spectrum(s)) {
        return use_magenta_marker ? MAGENTA_NAN_MARKER : make_zero_spectrum();
    }
    return s;
}

inline Spectrum clamp_throughput_update(const Spectrum &contrib) {
    Spectrum result = contrib;
    for (int i = 0; i < 3; ++i) {
        result[i] = std::clamp(result[i], Real(0), MAX_THROUGHPUT_UPDATE);
    }
    return result;
}

// v36 HARD FIX: Clamp Monte Carlo weight = bsdf_val / pdf
// This is the ROOT FIX for TT lobe fireflies where PDF approaches infinity
// causing weight explosion. Applied BEFORE any other processing.
// v36: Now also handles NaN/Inf from divide-by-zero
inline Spectrum clamp_mc_weight(const Spectrum &bsdf_val, Real pdf) {
    // v36: Reject zero/negative/NaN/Inf PDF
    if (pdf <= Real(1e-8) || std::isnan(pdf) || std::isinf(pdf)) {
        return make_zero_spectrum();
    }

    // v36: Reject NaN/Inf in BSDF value BEFORE division
    if (!is_valid_spectrum(bsdf_val)) {
        return make_zero_spectrum();
    }

    Spectrum weight = bsdf_val / pdf;

    // v36: Reject NaN/Inf AFTER division (can happen from 0/0 or Inf/Inf)
    if (!is_valid_spectrum(weight)) {
        return make_zero_spectrum();
    }

    for (int i = 0; i < 3; ++i) {
        weight[i] = std::clamp(weight[i], Real(0), MAX_MC_WEIGHT);
    }
    return weight;
}

// v31: Bounce-aware BSDF contribution clamping
// - Primary (bounce 0): Higher limit for sharp specular
// - Indirect (bounce > 0): Lower limit to prevent fireflies
inline Spectrum clamp_bsdf_contribution(const Spectrum &contrib, Real roughness, int bounce) {
    // Diffuse materials (roughness >= 0.5) don't need clamping
    if (roughness >= Real(0.5)) {
        return contrib;
    }

    // Select clamp limit based on bounce depth
    Real clamp_limit = (bounce == 0) ? MAX_BSDF_CONTRIBUTION_PRIMARY : MAX_BSDF_CONTRIBUTION_INDIRECT;

    // Interpolate clamp limit based on roughness (smoother transition)
    // roughness 0 -> full clamp, roughness 0.5 -> no clamp
    Real blend = std::clamp(roughness * Real(2), Real(0), Real(1));
    clamp_limit = clamp_limit + (Real(1000) - clamp_limit) * blend;  // Relax limit for rougher materials

    Spectrum result = contrib;
    for (int i = 0; i < 3; ++i) {
        result[i] = std::clamp(result[i], Real(0), clamp_limit);
    }
    return result;
}

// v31: Soft NEE clamp for highly specular materials
// Returns clamped contribution if material is very glossy, otherwise passthrough
inline Spectrum clamp_nee_contribution(const Spectrum &contrib, Real roughness) {
    // Only clamp VERY specular materials (roughness < 0.1)
    // This prevents "white pepper" noise from direct sun on wet oil film
    if (roughness >= Real(0.1)) {
        return contrib;  // Pass through for non-specular
    }

    Spectrum result = contrib;
    for (int i = 0; i < 3; ++i) {
        result[i] = std::clamp(result[i], Real(0), MAX_NEE_SPECULAR);
    }
    return result;
}

// v31: Apply path space filtering - increase effective roughness on indirect bounces
inline Real apply_path_space_filter(Real roughness, int bounce) {
    if (bounce == 0) {
        return roughness;  // Keep original for primary
    }
    // Force minimum roughness on indirect bounces
    return max(roughness, PATH_SPACE_MIN_ROUGHNESS);
}

/// Unidirectional path tracing
Spectrum path_tracing(const Scene &scene,
                      int x, int y, /* pixel coordinates */
                      pcg32_state &rng) {
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = init_ray_differential(w, h);

    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
    if (!vertex_) {
        // Hit background. Account for the environment map if needed.
        if (has_envmap(scene)) {
            const Light &envmap = get_envmap(scene);
            return emission(envmap,
                            -ray.dir, // pointing outwards from light
                            ray_diff.spread,
                            PointAndNormal{}, // dummy parameter for envmap
                            scene);
        }
        return make_zero_spectrum();
    }
    PathVertex vertex = *vertex_;

    Spectrum radiance = make_zero_spectrum();
    // A path's contribution is 
    // C(v) = W(v0, v1) * G(v0, v1) * f(v0, v1, v2) * 
    //                    G(v1, v2) * f(v1, v2, v3) * 
    //                  ........
    //                  * G(v_{n-1}, v_n) * L(v_{n-1}, v_n)
    // where v is the path vertices, W is the sensor response
    // G is the geometry term, f is the BSDF, L is the emission
    //
    // "sample_primary" importance samples both W and G,
    // and we assume it always has weight 1.

    // current_path_throughput stores the ratio between
    // 1) the path contribution from v0 up to v_{i} (the BSDF f(v_{i-1}, v_i, v_{i+1}) is not included), 
    // where i is where the PathVertex "vertex" lies on, and
    // 2) the probability density for computing the path v from v0 up to v_i,
    // so that we can compute the Monte Carlo estimates C/p. 
    Spectrum current_path_throughput = fromRGB(Vector3{1, 1, 1});
    // eta_scale stores the scale introduced by Snell-Descartes law to the BSDF (eta^2).
    // We use the same Russian roulette strategy as Mitsuba/pbrt-v3
    // and tracking eta_scale and removing it from the
    // path contribution is crucial for many bounces of refraction.
    Real eta_scale = Real(1);

    // We hit a light immediately. 
    // This path has only two vertices and has contribution
    // C = W(v0, v1) * G(v0, v1) * L(v0, v1)
    if (is_light(scene.shapes[vertex.shape_id])) {
        radiance += current_path_throughput *
            emission(vertex, -ray.dir, scene);
    }

    // We iteratively sum up path contributions from paths with different number of vertices
    // If max_depth == -1, we rely on Russian roulette for path termination.
    int max_depth = scene.options.max_depth;
    Real current_roughness = Real(1);  // Track roughness of current surface for clamping decisions
    for (int num_vertices = 3; max_depth == -1 || num_vertices <= max_depth + 1; num_vertices++) {
        // v31: Track bounce index for path space filtering
        int bounce = num_vertices - 3;  // bounce 0 = first surface hit

        // v32 CRITICAL: Set bounce_depth on vertex BEFORE any BSDF evaluation!
        // This enables BSDFs to apply path space filtering (roughness clamping)
        // based on whether this is a primary ray (bounce=0) or indirect (bounce>0)
        vertex.bounce_depth = bounce;

        // We are at v_i, and all the path contribution on and before has been accounted for.
        // Now we need to somehow generate v_{i+1} to account for paths with more vertices.
        // In path tracing, we generate two vertices:
        // 1) we sample a point on the light source (often called "Next Event Estimation")
        // 2) we randomly trace a ray from the surface point at v_i and hope we hit something.
        //
        // The first importance samples L(v_i, v_{i+1}), and the second
        // importance samples f(v_{i-1}, v_i, v_{i+1}) * G(v_i, v_{i+1})
        //
        // We then combine the two sampling strategies to estimate the contribution using weighted average.
        // Say the contribution of the first sampling is C1 (with probability density p1), 
        // and the contribution of the second sampling is C2 (with probability density p2,
        // then we compute the estimate as w1*C1/p1 + w2*C2/p2.
        //
        // Assuming the vertices for C1 is v^1, and v^2 for C2,
        // Eric Veach showed that it is a good idea setting 
        // w1 = p_1(v^1)^k / (p_1(v^1)^k + p_2(v^1)^k)
        // w2 = p_2(v^2)^k / (p_1(v^2)^k + p_2(v^2)^k),
        // where k is some scalar real number, and p_a(v^b) is the probability density of generating
        // vertices v^b using sampling method "a".
        // We will set k=2 as suggested by Eric Veach.

        // Finally, we set our "next vertex" in the loop to the v_{i+1} generated
        // by the second sampling, and update current_path_throughput using
        // our hemisphere sampling.

        // Let's implement this!
        const Material &mat = scene.materials[vertex.material_id];

        // First, we sample a point on the light source.
        // We do this by first picking a light source, then pick a point on it.
        Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real light_w = next_pcg32_real<Real>(rng);
        Real shape_w = next_pcg32_real<Real>(rng);
        int light_id = sample_light(scene, light_w);
        const Light &light = scene.lights[light_id];
        PointAndNormal point_on_light =
            sample_point_on_light(light, vertex.position, light_uv, shape_w, scene);

        // Next, we compute w1*C1/p1. We store C1/p1 in C1.
        Spectrum C1 = make_zero_spectrum();
        Real w1 = 0;
        // Remember "current_path_throughput" already stores all the path contribution on and before v_i.
        // So we only need to compute G(v_{i}, v_{i+1}) * f(v_{i-1}, v_{i}, v_{i+1}) * L(v_{i}, v_{i+1})
        {
            // Let's first deal with C1 = G * f * L.
            // Let's first compute G.
            Real G = 0;
            Vector3 dir_light;
            // The geometry term is different between directional light sources and
            // others. Currently we only have environment maps as directional light sources.
            if (!is_envmap(light)) {
                dir_light = normalize(point_on_light.position - vertex.position);
                // If the point on light is occluded, G is 0. So we need to test for occlusion.
                // To avoid self intersection, we need to set the tnear of the ray
                // to a small "epsilon". We set the epsilon to be a small constant times the
                // scale of the scene, which we can obtain through the get_shadow_epsilon() function.
                // v37: Use curve-aware epsilon for micro-cylinder self-intersection
                Real shadow_eps = get_shadow_epsilon_for_vertex(scene, vertex);
                Ray shadow_ray{vertex.position, dir_light,
                               shadow_eps,
                               (1 - shadow_eps) *
                                   distance(point_on_light.position, vertex.position)};
                // Pass origin shape_id to handle shadow-invisible curves (wet fur)
                if (!occluded(scene, shadow_ray, vertex.shape_id)) {
                    // geometry term is cosine at v_{i+1} divided by distance squared
                    // this can be derived by the infinitesimal area of a surface projected on
                    // a unit sphere -- it's the Jacobian between the area measure and the solid angle
                    // measure.
                    G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                        distance_squared(point_on_light.position, vertex.position);
                }
            } else {
                // The direction from envmap towards the point is stored in
                // point_on_light.normal.
                dir_light = -point_on_light.normal;
                // If the point on light is occluded, G is 0. So we need to test for occlusion.
                // To avoid self intersection, we need to set the tnear of the ray
                // to a small "epsilon" which we define as c_shadow_epsilon as a global constant.
                // v37: Use curve-aware epsilon for micro-cylinder self-intersection
                Ray shadow_ray{vertex.position, dir_light,
                               get_shadow_epsilon_for_vertex(scene, vertex),
                               infinity<Real>() /* envmaps are infinitely far away */};
                // Pass origin shape_id to handle shadow-invisible curves (wet fur)
                if (!occluded(scene, shadow_ray, vertex.shape_id)) {
                    // We integrate envmaps using the solid angle measure,
                    // so the geometry term is 1.
                    G = 1;
                }
            }

            // Before we proceed, we first compute the probability density p1(v1)
            // The probability density for light sampling to sample our point is
            // just the probability of sampling a light times the probability of sampling a point
            Real p1 = light_pmf(scene, light_id) *
                pdf_point_on_light(light, point_on_light, vertex.position, scene);

            // We don't need to continue the computation if G is 0.
            // Also sometimes there can be some numerical issue such that we generate
            // a light path with probability zero
            if (G > 0 && p1 > 0) {
                // Let's compute f (BSDF) next.
                Vector3 dir_view = -ray.dir;
                assert(vertex.material_id >= 0);
                Spectrum f = eval(mat, dir_view, dir_light, vertex, scene.texture_pool);

                // Evaluate the emission
                // We set the footprint to zero since it is not fully clear how
                // to set it in this case.
                // One way is to use a roughness based heuristics, but we have multi-layered BRDFs.
                // See "Real-time Shading with Filtered Importance Sampling" from Colbert et al.
                // for the roughness based heuristics.
                Spectrum L = emission(light, -dir_light, Real(0), point_on_light, scene);
                // v38: Clamp radiance on indirect bounces - kills HDRI sun fireflies in NEE
                L = clamp_indirect_radiance(L, bounce);

                // C1 is just a product of all of them!
                C1 = G * f * L;
            
                // Next let's compute w1

                // Remember that we want to set
                // w1 = p_1(v^1)^2 / (p_1(v^1)^2 + p_2(v^1)^2)
                // Notice that all of the probability density share the same path prefix and those cancel out.
                // Therefore we only need to account for the generation of the vertex v_{i+1}.

                // The probability density for our hemispherical sampling to sample 
                Real p2 = pdf_sample_bsdf(
                    mat, dir_view, dir_light, vertex, scene.texture_pool);
                // !!!! IMPORTANT !!!!
                // In general, p1 and p2 now live in different spaces!!
                // our BSDF API outputs a probability density in the solid angle measure
                // while our light probability density is in the area measure.
                // We need to make sure that they are in the same space.
                // This can be done by accounting for the Jacobian of the transformation
                // between the two measures.
                // In general, I recommend to transform everything to area measure 
                // (except for directional lights) since it fits to the path-space math better.
                // Converting a solid angle measure to an area measure is just a
                // multiplication of the geometry term G (let solid angle be dS, area be dA,
                // we have dA/dS = G).
                p2 *= G;

                w1 = (p1*p1) / (p1*p1 + p2*p2);
                C1 /= p1;
            }
        }
        // v31: Soft NEE clamp for highly specular materials (roughness < 0.1)
        // Preserves wet oil sheen while preventing "white pepper" noise
        Spectrum nee_contrib = current_path_throughput * C1 * w1;
        radiance += clamp_nee_contribution(nee_contrib, current_roughness);

        // Let's do the hemispherical sampling next.
        Vector3 dir_view = -ray.dir;
        Vector2 bsdf_rnd_param_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real bsdf_rnd_param_w = next_pcg32_real<Real>(rng);
        std::optional<BSDFSampleRecord> bsdf_sample_ =
            sample_bsdf(mat,
                        dir_view,
                        vertex,
                        scene.texture_pool,
                        bsdf_rnd_param_uv,
                        bsdf_rnd_param_w);
        if (!bsdf_sample_) {
            // BSDF sampling failed. Abort the loop.
            break;
        }
        const BSDFSampleRecord &bsdf_sample = *bsdf_sample_;
        Vector3 dir_bsdf = bsdf_sample.dir_out;
        // Update ray differentials & eta_scale
        if (bsdf_sample.eta == 0) {
            ray_diff.spread = reflect(ray_diff, vertex.mean_curvature, bsdf_sample.roughness);
        } else {
            ray_diff.spread = refract(ray_diff, vertex.mean_curvature, bsdf_sample.eta, bsdf_sample.roughness);
            eta_scale /= (bsdf_sample.eta * bsdf_sample.eta);
        }

        // Trace a ray towards bsdf_dir. Note that again we have
        // to have an "epsilon" tnear to prevent self intersection.
        // v37: Use curve-aware epsilon to escape micro-cylinder self-intersection
        Ray bsdf_ray{vertex.position, dir_bsdf, get_intersection_epsilon_for_vertex(scene, vertex), infinity<Real>()};
        std::optional<PathVertex> bsdf_vertex = intersect(scene, bsdf_ray);

        // To update current_path_throughput
        // we need to multiply G(v_{i}, v_{i+1}) * f(v_{i-1}, v_{i}, v_{i+1}) to it
        // and divide it with the pdf for getting v_{i+1} using hemisphere sampling.
        Real G;
        if (bsdf_vertex) {
            // =================================================================
            // v37 GEOMETRY TERM SAFEGUARD
            // =================================================================
            // For micro-cylinder curves (radius ~0.0001), self-intersection or
            // adjacent-strand hits at very small distances cause G to explode.
            // We add multiple safeguards:
            // 1. Minimum distance: If hit is too close, treat as self-intersection
            // 2. Maximum G clamp: Prevent infinite values from entering MC estimator
            // 3. NaN/Inf check: Reject degenerate geometry configurations
            // =================================================================
            Real dist_sq = distance_squared(bsdf_vertex->position, vertex.position);
            Real cos_theta = fabs(dot(dir_bsdf, bsdf_vertex->geometric_normal));

            // v37: Minimum distance based on curve geometry
            // If we hit something closer than our escape epsilon, it's self-intersection
            Real min_dist_sq = get_intersection_epsilon_for_vertex(scene, vertex);
            min_dist_sq = min_dist_sq * min_dist_sq;  // Square it for comparison

            if (dist_sq < min_dist_sq) {
                // Self-intersection detected - use safe G value
                G = Real(1);
            } else {
                G = cos_theta / dist_sq;

                // v38: Clamp G to prevent explosion
                // v37 used 1e6 which is absurdly high - 100 is already extreme
                if (G > MAX_GEOMETRY_TERM || std::isnan(G) || std::isinf(G)) {
                    G = MAX_GEOMETRY_TERM;
                }
            }
        } else {
            // We hit nothing, set G to 1 to account for the environment map contribution.
            G = 1;
        }

        Spectrum f = eval(mat, dir_view, dir_bsdf, vertex, scene.texture_pool);
        Real p2 = pdf_sample_bsdf(mat, dir_view, dir_bsdf, vertex, scene.texture_pool);
        if (p2 <= 0) {
            // Numerical issue -- we generated some invalid rays.
            break;
        }

        // Remember to convert p2 to area measure!
        p2 *= G;
        // note that G cancels out in the division f/p, but we still need
        // G later for the calculation of w2.

        // Now we want to check whether dir_bsdf hit a light source, and
        // account for the light contribution (C2 & w2 & p2).
        // There are two possibilities: either we hit an emissive surface,
        // or we hit an environment map.
        // We will handle them separately.
        if (bsdf_vertex && is_light(scene.shapes[bsdf_vertex->shape_id])) {
            // G & f are already computed.
            Spectrum L = emission(*bsdf_vertex, -dir_bsdf, scene);
            // v38: Clamp radiance on indirect bounces - kills HDRI sun fireflies
            L = clamp_indirect_radiance(L, bounce);
            Spectrum C2 = G * f * L;
            // Next let's compute p1(v2): the probability of the light source sampling
            // directly drawing the point corresponds to bsdf_dir.
            int light_id = get_area_light_id(scene.shapes[bsdf_vertex->shape_id]);
            assert(light_id >= 0);
            const Light &light = scene.lights[light_id];
            PointAndNormal light_point{bsdf_vertex->position, bsdf_vertex->geometric_normal};
            Real p1 = light_pmf(scene, light_id) *
                pdf_point_on_light(light, light_point, vertex.position, scene);
            Real w2 = (p2*p2) / (p1*p1 + p2*p2);

            C2 /= p2;
            // v31: Bounce-aware BSDF contribution clamping
            Spectrum contrib = current_path_throughput * C2 * w2;
            radiance += clamp_bsdf_contribution(contrib, bsdf_sample.roughness, bounce);
        } else if (!bsdf_vertex && has_envmap(scene)) {
            // G & f are already computed.
            const Light &light = get_envmap(scene);
            Spectrum L = emission(light,
                                  -dir_bsdf, // pointing outwards from light
                                  ray_diff.spread,
                                  PointAndNormal{}, // dummy parameter for envmap
                                  scene);
            // v38: Clamp radiance on indirect bounces - kills HDRI sun fireflies
            L = clamp_indirect_radiance(L, bounce);
            Spectrum C2 = G * f * L;
            // Next let's compute p1(v2): the probability of the light source sampling
            // directly drawing the direction bsdf_dir.
            PointAndNormal light_point{Vector3{0, 0, 0}, -dir_bsdf}; // pointing outwards from light
            Real p1 = light_pmf(scene, scene.envmap_light_id) *
                      pdf_point_on_light(light, light_point, vertex.position, scene);
            Real w2 = (p2*p2) / (p1*p1 + p2*p2);

            C2 /= p2;
            // v31: Bounce-aware BSDF contribution clamping
            Spectrum contrib = current_path_throughput * C2 * w2;
            radiance += clamp_bsdf_contribution(contrib, bsdf_sample.roughness, bounce);
        }

        if (!bsdf_vertex) {
            // Hit nothing -- can't continue tracing.
            break;
        }

        // Update rays/intersection/current_path_throughput/current_pdf
        // Russian roulette heuristics
        Real rr_prob = 1;
        if (num_vertices - 1 >= scene.options.rr_depth) {
            rr_prob = min(max((1 / eta_scale) * current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                // Terminate the path
                break;
            }
        }

        ray = bsdf_ray;
        vertex = *bsdf_vertex;
        // v31: Track roughness for next iteration's NEE clamping decision
        current_roughness = bsdf_sample.roughness;

        // =========================================================================
        // v34 HARD FIX: Clamp Monte Carlo weight FIRST
        // =========================================================================
        // The TT lobe PDF can be extremely small (approaching 0) causing
        // bsdf_val / pdf to explode. This is the mathematical root cause of fireflies.
        // We clamp the raw MC weight = f / p2 BEFORE multiplying by G / rr_prob.
        // =========================================================================
        Spectrum mc_weight = clamp_mc_weight(f, p2);  // Hard clamp: f/p2 <= 10
        Spectrum throughput_update = (G * mc_weight) / rr_prob;

        // Additional throughput clamp for safety (prevents exponential buildup)
        throughput_update = clamp_throughput_update(throughput_update);

        // v36: Sanitize throughput update - kill NaN/Inf before they propagate
        throughput_update = sanitize_spectrum(throughput_update);
        current_path_throughput = current_path_throughput * throughput_update;

        // v36: Also sanitize accumulated throughput
        current_path_throughput = sanitize_spectrum(current_path_throughput);
    }

    // =========================================================================
    // v39 FINAL SAMPLE CLAMP - THE INDUSTRY STANDARD "CATCH-ALL"
    // =========================================================================
    // This is the ultimate gatekeeper that production renderers use.
    // No matter what multiplication chains occurred in the path, this ensures
    // no single sample can exceed MAX_SAMPLE_VALUE (20.0).
    //
    // Applied AFTER all path tracing math, BEFORE returning to the accumulator.
    // This mathematically guarantees firefly elimination.
    // =========================================================================
    return clamp_final_sample(radiance);
}
