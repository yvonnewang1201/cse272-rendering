#pragma once

#include <cmath>

// ============================================================================
// Equiangular Sampling for Volumetric Path Tracing
// Reference: Kulla & Fajardo, "Importance Sampling Techniques for Path Tracing
//            in Participating Media", EGSR 2012
// ============================================================================

// Structure to hold equiangular sampling parameters
struct EquiangularSample {
    Real t;           // Sampled distance along ray
    Real pdf;         // PDF of the sample
    bool valid;       // Whether sampling was successful
};

// Sample distance using equiangular distribution
// ray_org: ray origin
// ray_dir: ray direction (normalized)
// light_pos: position of the point/area light (use center for area lights)
// t_min: minimum valid distance (usually 0 or small epsilon)
// t_max: maximum valid distance (distance to surface hit)
// u: uniform random number in [0, 1]
inline EquiangularSample sample_equiangular(
    const Vector3 &ray_org,
    const Vector3 &ray_dir,
    const Vector3 &light_pos,
    Real t_min,
    Real t_max,
    Real u
) {
    EquiangularSample result;
    result.valid = false;
    result.t = 0;
    result.pdf = 0;

    // Project light position onto ray to find closest point
    Vector3 delta = light_pos - ray_org;
    Real t_proj = dot(delta, ray_dir);

    // Compute perpendicular distance from light to ray
    Vector3 closest_point = ray_org + t_proj * ray_dir;
    Real D = distance(light_pos, closest_point);

    // Handle degenerate case where light is on the ray
    if (D < 1e-6) {
        // Fall back to uniform sampling in this case
        result.t = t_min + u * (t_max - t_min);
        result.pdf = Real(1) / (t_max - t_min);
        result.valid = (t_max > t_min);
        return result;
    }

    // Compute angular bounds
    Real theta_a = std::atan((t_min - t_proj) / D);
    Real theta_b = std::atan((t_max - t_proj) / D);

    // Handle case where bounds are too close
    if (std::abs(theta_b - theta_a) < 1e-6) {
        result.t = (t_min + t_max) * Real(0.5);
        result.pdf = Real(1) / (t_max - t_min);
        result.valid = true;
        return result;
    }

    // Sample angle uniformly between theta_a and theta_b
    Real theta = theta_a + u * (theta_b - theta_a);

    // Convert angle back to distance
    result.t = t_proj + D * std::tan(theta);

    // Clamp to valid range (numerical safety)
    result.t = std::max(t_min, std::min(t_max, result.t));

    // Compute PDF: D / ((theta_b - theta_a) * (D^2 + (t - t_proj)^2))
    Real dt = result.t - t_proj;
    result.pdf = D / ((theta_b - theta_a) * (D * D + dt * dt));

    result.valid = true;
    return result;
}

// Compute the PDF of equiangular sampling for a given distance t
inline Real pdf_equiangular(
    const Vector3 &ray_org,
    const Vector3 &ray_dir,
    const Vector3 &light_pos,
    Real t_min,
    Real t_max,
    Real t
) {
    // Project light position onto ray
    Vector3 delta = light_pos - ray_org;
    Real t_proj = dot(delta, ray_dir);

    // Compute perpendicular distance
    Vector3 closest_point = ray_org + t_proj * ray_dir;
    Real D = distance(light_pos, closest_point);

    // Handle degenerate case
    if (D < 1e-6) {
        return Real(1) / (t_max - t_min);
    }

    // Compute angular bounds
    Real theta_a = std::atan((t_min - t_proj) / D);
    Real theta_b = std::atan((t_max - t_proj) / D);

    if (std::abs(theta_b - theta_a) < 1e-6) {
        return Real(1) / (t_max - t_min);
    }

    // PDF: D / ((theta_b - theta_a) * (D^2 + (t - t_proj)^2))
    Real dt = t - t_proj;
    return D / ((theta_b - theta_a) * (D * D + dt * dt));
}

// ============================================================================

// The simplest volumetric renderer:
// single absorption only homogeneous volume
// only handle directly visible light sources
Spectrum vol_path_tracing_1(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    // Intersect with scene
    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);

    if (!vertex_) {
        // No intersection - return zero (no environment map handling)
        return make_zero_spectrum();
    }

    PathVertex vertex = *vertex_;
    Real t_hit = distance(ray.org, vertex.position);

    // Compute transmittance: exp(-sigma_a * t_hit)
    // The camera medium_id tells us which medium we're in
    Spectrum transmittance = make_const_spectrum(1);
    int current_medium_id = scene.camera.medium_id;

    if (current_medium_id >= 0) {
        // We're inside a medium
        const Medium &medium = scene.media[current_medium_id];
        Spectrum sigma_a = get_sigma_a(medium, vertex.position);
        transmittance = exp(-sigma_a * t_hit);
    }

    // Check if we hit a light source
    Spectrum Le = make_zero_spectrum();
    if (is_light(scene.shapes[vertex.shape_id])) {
        Le = emission(vertex, -ray.dir, scene);
    }

    return transmittance * Le;
}

// The second simplest volumetric renderer:
// single monochromatic homogeneous volume with single scattering,
// no need to handle surface lighting, only directly visible light source
// NOW WITH EQUIANGULAR SAMPLING + MIS (Bonus implementation)
Spectrum vol_path_tracing_2(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    // Get current medium
    int current_medium_id = scene.camera.medium_id;
    if (current_medium_id < 0) {
        // Not in a medium - fall back to simple path tracing behavior
        std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
        if (!vertex_) return make_zero_spectrum();
        PathVertex vertex = *vertex_;
        if (is_light(scene.shapes[vertex.shape_id])) {
            return emission(vertex, -ray.dir, scene);
        }
        return make_zero_spectrum();
    }

    const Medium &medium = scene.media[current_medium_id];
    Spectrum sigma_a = get_sigma_a(medium, Vector3{0, 0, 0});
    Spectrum sigma_s = get_sigma_s(medium, Vector3{0, 0, 0});
    Spectrum sigma_t = sigma_a + sigma_s;

    // For monochromatic medium, use the first channel
    Real sigma_t_scalar = sigma_t[0];

    // Intersect with scene to find t_hit
    std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
    Real t_hit = infinity<Real>();
    if (vertex_) {
        t_hit = distance(ray.org, vertex_->position);
    }

    // First, sample a light to get its position for equiangular sampling
    Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
    Real light_w = next_pcg32_real<Real>(rng);
    Real shape_w = next_pcg32_real<Real>(rng);
    int light_id = sample_light(scene, light_w);
    const Light &light = scene.lights[light_id];

    // Get a preliminary point on light (we'll resample after getting scatter point)
    PointAndNormal point_on_light = sample_point_on_light(light, ray.org, light_uv, shape_w, scene);
    Vector3 light_pos = point_on_light.position;

    // ============ MIS between transmittance and equiangular sampling ============
    // Use one-sample MIS: randomly choose which sampling strategy to use
    Real strategy_selector = next_pcg32_real<Real>(rng);
    Real u_dist = next_pcg32_real<Real>(rng);

    Real t;
    Real pdf_trans, pdf_equi;
    bool use_equiangular = (strategy_selector < Real(0.5));

    // Minimum distance for sampling (avoid self-intersection)
    Real t_min = get_shadow_epsilon(scene);

    if (use_equiangular && t_hit > t_min) {
        // Sample using equiangular distribution
        EquiangularSample eq_sample = sample_equiangular(
            ray.org, ray.dir, light_pos, t_min, t_hit, u_dist);

        if (!eq_sample.valid) {
            // Fall back to transmittance sampling
            t = -log(1 - u_dist) / sigma_t_scalar;
            pdf_equi = Real(0);
            pdf_trans = exp(-sigma_t_scalar * t) * sigma_t_scalar;
        } else {
            t = eq_sample.t;
            pdf_equi = eq_sample.pdf;
            // Also compute transmittance PDF at this distance
            pdf_trans = exp(-sigma_t_scalar * t) * sigma_t_scalar;
        }
    } else {
        // Sample using transmittance
        t = -log(1 - u_dist) / sigma_t_scalar;
        pdf_trans = exp(-sigma_t_scalar * t) * sigma_t_scalar;
        // Compute equiangular PDF at this distance
        if (t < t_hit && t > t_min) {
            pdf_equi = pdf_equiangular(ray.org, ray.dir, light_pos, t_min, t_hit, t);
        } else {
            pdf_equi = Real(0);
        }
    }

    if (t < t_hit) {
        // Scatter in the medium
        Spectrum transmittance = exp(-sigma_t * t);

        // Position where scattering occurs
        Vector3 p = ray.org + t * ray.dir;

        // Resample point on light from the scattering position for better results
        Vector2 light_uv2{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
        Real shape_w2 = next_pcg32_real<Real>(rng);
        point_on_light = sample_point_on_light(light, p, light_uv2, shape_w2, scene);

        // Compute direction to light
        Vector3 dir_light = normalize(point_on_light.position - p);
        Real dist_to_light = distance(point_on_light.position, p);

        // Check visibility (shadow ray)
        Ray shadow_ray{p, dir_light, get_shadow_epsilon(scene),
                       (1 - get_shadow_epsilon(scene)) * dist_to_light};

        // No origin shape for medium scattering, pass -1
        if (!occluded(scene, shadow_ray, -1)) {
            // Compute geometry term: |ω' · n_p'| / ||p - p'||²
            Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                     (dist_to_light * dist_to_light);

            // Transmittance from p to light: exp(-sigma_t * dist_to_light)
            Spectrum T_light = exp(-sigma_t * dist_to_light);

            // Phase function evaluation
            PhaseFunction phase_func = get_phase_function(medium);
            Spectrum phase_val = eval(phase_func, -ray.dir, dir_light);

            // Light emission
            Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

            // PDF for sampling the light
            Real pdf_light = light_pmf(scene, light_id) *
                             pdf_point_on_light(light, point_on_light, p, scene);

            if (pdf_light > 0) {
                // Compute MIS weight using balance heuristic
                // Combined PDF = 0.5 * pdf_trans + 0.5 * pdf_equi
                Real combined_pdf = Real(0.5) * pdf_trans + Real(0.5) * pdf_equi;

                if (combined_pdf > 0) {
                    // L_scatter1 = phase * Le * T_light * G / pdf_light
                    Spectrum L_s1 = phase_val * Le * T_light * G / pdf_light;

                    // Final contribution with MIS
                    // transmittance * sigma_s * L_s1 / combined_pdf
                    return transmittance * sigma_s * L_s1 / combined_pdf;
                }
            }
        }
        return make_zero_spectrum();
    } else {
        // Hit a surface
        if (!vertex_) return make_zero_spectrum();

        PathVertex vertex = *vertex_;

        // For surface hits, only transmittance sampling contributes
        // trans_pdf is probability of NOT scattering before t_hit: exp(-sigma_t * t_hit)
        Real trans_pdf = exp(-sigma_t_scalar * t_hit);
        Spectrum transmittance = exp(-sigma_t * t_hit);

        // Check if we hit a light source
        Spectrum Le = make_zero_spectrum();
        if (is_light(scene.shapes[vertex.shape_id])) {
            Le = emission(vertex, -ray.dir, scene);
        }

        return (transmittance / trans_pdf) * Le;
    }
}

// Helper function to update medium when crossing a surface
inline int update_medium(const Ray &ray, const PathVertex &vertex, int current_medium_id) {
    // Only update if there's a medium transition
    if (vertex.interior_medium_id != vertex.exterior_medium_id) {
        if (dot(ray.dir, vertex.geometric_normal) > 0) {
            // Exiting: use exterior medium
            return vertex.exterior_medium_id;
        } else {
            // Entering: use interior medium
            return vertex.interior_medium_id;
        }
    }
    return current_medium_id;
}

// The third volumetric renderer (not so simple anymore):
// multiple monochromatic homogeneous volumes with multiple scattering
// no need to handle surface lighting, only directly visible light source
Spectrum vol_path_tracing_3(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    int current_medium_id = scene.camera.medium_id;
    Spectrum current_path_throughput = make_const_spectrum(1);
    Spectrum radiance = make_zero_spectrum();
    int bounces = 0;
    int max_depth = scene.options.max_depth;
    int rr_depth = scene.options.rr_depth;

    while (true) {
        bool scatter = false;

        // Intersect with scene
        std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
        Real t_hit = infinity<Real>();
        if (vertex_) {
            t_hit = distance(ray.org, vertex_->position);
        }

        Spectrum transmittance = make_const_spectrum(1);
        Real trans_pdf = 1;
        Real t = t_hit; // Distance traveled

        if (current_medium_id >= 0) {
            // We're in a medium - sample distance
            const Medium &medium = scene.media[current_medium_id];
            Spectrum sigma_a = get_sigma_a(medium, ray.org);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);
            Spectrum sigma_t = sigma_a + sigma_s;
            Real sigma_t_scalar = sigma_t[0]; // Monochromatic

            if (sigma_t_scalar > 0) {
                // Sample distance: t = -log(1-u) / sigma_t
                Real u = next_pcg32_real<Real>(rng);
                Real sampled_t = -log(1 - u) / sigma_t_scalar;

                if (sampled_t < t_hit) {
                    // Scatter in the medium
                    scatter = true;
                    t = sampled_t;
                    trans_pdf = exp(-sigma_t_scalar * t) * sigma_t_scalar;
                    transmittance = exp(-sigma_t * t);
                } else {
                    // Hit surface
                    t = t_hit;
                    trans_pdf = exp(-sigma_t_scalar * t_hit);
                    transmittance = exp(-sigma_t * t_hit);
                }
            }
        }

        // Update ray origin
        ray.org = ray.org + t * ray.dir;

        // Update path throughput
        current_path_throughput *= (transmittance / trans_pdf);

        // If we didn't scatter, we hit a surface (or nothing)
        if (!scatter) {
            if (vertex_) {
                // Include emission from the surface
                if (is_light(scene.shapes[vertex_->shape_id])) {
                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene);
                }
            }
            // No surface hit means we're done
            if (!vertex_) {
                break;
            }
        }

        // Check max depth
        if (max_depth != -1 && bounces >= max_depth - 1) {
            break;
        }

        // Handle index-matching surfaces or scattering
        if (!scatter && vertex_) {
            if (vertex_->material_id == -1) {
                // Index-matching surface - pass through
                current_medium_id = update_medium(ray, *vertex_, current_medium_id);
                // Slightly offset to avoid self-intersection
                ray.org = ray.org + get_intersection_epsilon(scene) * ray.dir;
                bounces++;
                continue;
            } else {
                // Hit an opaque surface - no surface lighting in vol_path_tracing_3
                break;
            }
        }

        // Scatter: sample new direction from phase function
        if (scatter) {
            const Medium &medium = scene.media[current_medium_id];
            PhaseFunction phase_func = get_phase_function(medium);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);

            Vector2 phase_rnd{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            std::optional<Vector3> next_dir_ = sample_phase_function(phase_func, -ray.dir, phase_rnd);

            if (!next_dir_) {
                break;
            }

            Vector3 next_dir = *next_dir_;

            // Phase function value and PDF
            Spectrum phase_val = eval(phase_func, -ray.dir, next_dir);
            Real phase_pdf = pdf_sample_phase(phase_func, -ray.dir, next_dir);

            if (phase_pdf <= 0) {
                break;
            }

            // Update throughput: multiply by (phase / pdf) * sigma_s
            current_path_throughput *= (phase_val / phase_pdf) * sigma_s;

            // Update ray direction
            ray.dir = next_dir;
        }

        // Russian roulette
        Real rr_prob = 1;
        if (bounces >= rr_depth) {
            // Use max component of throughput for RR probability
            rr_prob = min(max(current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                break;
            }
            current_path_throughput /= rr_prob;
        }

        bounces++;
    }

    return radiance;
}

// The fourth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// still no surface lighting
Spectrum vol_path_tracing_4(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    int current_medium_id = scene.camera.medium_id;
    Spectrum current_path_throughput = make_const_spectrum(1);
    Spectrum radiance = make_zero_spectrum();
    int bounces = 0;
    int max_depth = scene.options.max_depth;
    int rr_depth = scene.options.rr_depth;

    // For MIS: cache quantities for computing PDF of alternative sampling
    Real dir_pdf = 0;              // PDF of phase function sampling (solid angle)
    Vector3 nee_p_cache = ray.org; // Position that can issue NEE
    Real multi_trans_pdf = 1;      // Product of transmittance PDFs through index-matching surfaces
    bool never_scatter = true;     // True if path has never scattered

    while (true) {
        bool scatter = false;

        // Intersect with scene
        std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
        Real t_hit = infinity<Real>();
        if (vertex_) {
            t_hit = distance(ray.org, vertex_->position);
        }

        Spectrum transmittance = make_const_spectrum(1);
        Real trans_pdf = 1;
        Real t = t_hit;

        if (current_medium_id >= 0) {
            const Medium &medium = scene.media[current_medium_id];
            Spectrum sigma_a = get_sigma_a(medium, ray.org);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);
            Spectrum sigma_t = sigma_a + sigma_s;
            Real sigma_t_scalar = sigma_t[0];

            if (sigma_t_scalar > 0) {
                // Sample distance using transmittance sampling: t = -log(1-u) / sigma_t
                Real u = next_pcg32_real<Real>(rng);
                Real sampled_t = -log(1 - u) / sigma_t_scalar;

                if (sampled_t < t_hit) {
                    // Scatter in the medium
                    scatter = true;
                    t = sampled_t;
                    trans_pdf = exp(-sigma_t_scalar * t) * sigma_t_scalar;
                    transmittance = exp(-sigma_t * t);
                } else {
                    // Hit surface
                    t = t_hit;
                    trans_pdf = exp(-sigma_t_scalar * t_hit);
                    transmittance = exp(-sigma_t * t_hit);
                }
            }
        }

        ray.org = ray.org + t * ray.dir;
        current_path_throughput *= (transmittance / trans_pdf);

        // Update multi_trans_pdf for MIS
        multi_trans_pdf *= trans_pdf;

        // If we hit a surface (didn't scatter), include emission with MIS
        if (!scatter && vertex_) {
            if (is_light(scene.shapes[vertex_->shape_id])) {
                if (never_scatter) {
                    // Direct hit - no MIS needed
                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene);
                } else {
                    // Need MIS weight for phase function sampling
                    int light_id = get_area_light_id(scene.shapes[vertex_->shape_id]);
                    const Light &light = scene.lights[light_id];
                    PointAndNormal light_point{vertex_->position, vertex_->geometric_normal};

                    Real pdf_nee = light_pmf(scene, light_id) *
                                   pdf_point_on_light(light, light_point, nee_p_cache, scene);

                    // Geometry term for converting phase PDF to area measure
                    Real dist_sq = distance_squared(nee_p_cache, vertex_->position);
                    Real G = max(dot(-ray.dir, vertex_->geometric_normal), Real(0)) / dist_sq;

                    // Phase function PDF in area measure
                    Real pdf_phase = dir_pdf * multi_trans_pdf * G;

                    // Power heuristic
                    Real w = (pdf_phase * pdf_phase) / (pdf_phase * pdf_phase + pdf_nee * pdf_nee);

                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene) * w;
                }
            }
        }

        if (!scatter && !vertex_) {
            break;
        }

        if (max_depth != -1 && bounces >= max_depth - 1) {
            break;
        }

        // Handle index-matching surfaces
        if (!scatter && vertex_) {
            if (vertex_->material_id == -1) {
                current_medium_id = update_medium(ray, *vertex_, current_medium_id);
                ray.org = ray.org + get_intersection_epsilon(scene) * ray.dir;
                bounces++;
                continue;
            } else {
                // Hit opaque surface - no surface lighting in vol_path_tracing_4
                break;
            }
        }

        // Scatter: do NEE and sample new direction
        if (scatter) {
            const Medium &medium = scene.media[current_medium_id];
            PhaseFunction phase_func = get_phase_function(medium);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);

            // ============ Next Event Estimation ============
            Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real light_w = next_pcg32_real<Real>(rng);
            Real shape_w = next_pcg32_real<Real>(rng);
            int light_id = sample_light(scene, light_w);
            const Light &light = scene.lights[light_id];
            PointAndNormal point_on_light = sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

            Vector3 dir_light = normalize(point_on_light.position - ray.org);
            Real full_dist = distance(point_on_light.position, ray.org);

            // Trace shadow ray through index-matching surfaces
            Spectrum T_light = make_const_spectrum(1);
            Real p_trans_dir = 1;  // For MIS: PDF of transmittance sampling
            int shadow_medium_id = current_medium_id;
            Vector3 shadow_pos = ray.org;
            int shadow_bounces = 0;
            bool blocked = false;

            while (true) {
                Ray shadow_ray{shadow_pos, dir_light, get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) * distance(shadow_pos, point_on_light.position)};
                std::optional<PathVertex> shadow_isect = intersect(scene, shadow_ray, ray_diff);

                Real next_t = distance(shadow_pos, point_on_light.position);
                if (shadow_isect) {
                    next_t = distance(shadow_pos, shadow_isect->position);
                }

                // Account for transmittance
                if (shadow_medium_id >= 0) {
                    const Medium &shadow_medium = scene.media[shadow_medium_id];
                    Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_pos) +
                                              get_sigma_s(shadow_medium, shadow_pos);
                    Real shadow_sigma_t_scalar = shadow_sigma_t[0];
                    T_light *= exp(-shadow_sigma_t * next_t);
                    p_trans_dir *= exp(-shadow_sigma_t_scalar * next_t);
                }

                if (!shadow_isect) {
                    // Reached the light
                    break;
                } else {
                    if (shadow_isect->material_id >= 0) {
                        // Blocked by opaque surface
                        blocked = true;
                        break;
                    }
                    // Index-matching surface - pass through
                    shadow_bounces++;
                    if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
                        blocked = true;
                        break;
                    }
                    shadow_medium_id = update_medium(
                        Ray{shadow_pos, dir_light}, *shadow_isect, shadow_medium_id);
                    shadow_pos = shadow_isect->position + get_intersection_epsilon(scene) * dir_light;
                }
            }

            if (!blocked && max(T_light) > 0) {
                // Compute geometry term
                Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                         (full_dist * full_dist);

                // Phase function value
                Spectrum phase_val = eval(phase_func, -ray.dir, dir_light);

                // Light emission
                Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

                // PDF for NEE
                Real pdf_nee = light_pmf(scene, light_id) *
                               pdf_point_on_light(light, point_on_light, ray.org, scene);

                if (pdf_nee > 0) {
                    Spectrum contrib = T_light * G * phase_val * Le / pdf_nee;

                    // MIS weight: compare with phase function sampling
                    Real pdf_phase = pdf_sample_phase(phase_func, -ray.dir, dir_light) * G * p_trans_dir;

                    Real w = (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);

                    radiance += current_path_throughput * sigma_s * contrib * w;
                }
            }

            // ============ Phase Function Sampling ============
            Vector2 phase_rnd{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            std::optional<Vector3> next_dir_ = sample_phase_function(phase_func, -ray.dir, phase_rnd);

            if (!next_dir_) {
                break;
            }

            Vector3 next_dir = *next_dir_;
            Spectrum phase_val = eval(phase_func, -ray.dir, next_dir);
            Real phase_pdf = pdf_sample_phase(phase_func, -ray.dir, next_dir);

            if (phase_pdf <= 0) {
                break;
            }

            // Update cached values for MIS
            dir_pdf = phase_pdf;
            nee_p_cache = ray.org;
            multi_trans_pdf = 1;  // Reset after scatter
            never_scatter = false;

            current_path_throughput *= (phase_val / phase_pdf) * sigma_s;
            ray.dir = next_dir;
        }

        // Russian roulette
        Real rr_prob = 1;
        if (bounces >= rr_depth) {
            rr_prob = min(max(current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                break;
            }
            current_path_throughput /= rr_prob;
        }

        bounces++;
    }

    return radiance;
}

// The fifth volumetric renderer:
// multiple monochromatic homogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing_5(const Scene &scene,
                            int x, int y, /* pixel coordinates */
                            pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    int current_medium_id = scene.camera.medium_id;
    Spectrum current_path_throughput = make_const_spectrum(1);
    Spectrum radiance = make_zero_spectrum();
    int bounces = 0;
    int max_depth = scene.options.max_depth;
    int rr_depth = scene.options.rr_depth;

    // For MIS: cache quantities for computing PDF of alternative sampling
    Real dir_pdf = 0;              // PDF of directional sampling (solid angle)
    Vector3 nee_p_cache = ray.org; // Position that can issue NEE
    Real multi_trans_pdf = 1;      // Product of transmittance PDFs
    bool never_scatter = true;     // True if path has never scattered (volume or surface)

    while (true) {
        bool scatter = false;

        // Intersect with scene
        std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
        Real t_hit = infinity<Real>();
        if (vertex_) {
            t_hit = distance(ray.org, vertex_->position);
        }

        Spectrum transmittance = make_const_spectrum(1);
        Real trans_pdf = 1;
        Real t = t_hit;

        if (current_medium_id >= 0) {
            const Medium &medium = scene.media[current_medium_id];
            Spectrum sigma_a = get_sigma_a(medium, ray.org);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);
            Spectrum sigma_t = sigma_a + sigma_s;
            Real sigma_t_scalar = sigma_t[0];

            if (sigma_t_scalar > 0) {
                // Sample distance using transmittance sampling: t = -log(1-u) / sigma_t
                Real u = next_pcg32_real<Real>(rng);
                Real sampled_t = -log(1 - u) / sigma_t_scalar;

                if (sampled_t < t_hit) {
                    // Scatter in the medium
                    scatter = true;
                    t = sampled_t;
                    trans_pdf = exp(-sigma_t_scalar * t) * sigma_t_scalar;
                    transmittance = exp(-sigma_t * t);
                } else {
                    // Hit surface
                    t = t_hit;
                    trans_pdf = exp(-sigma_t_scalar * t_hit);
                    transmittance = exp(-sigma_t * t_hit);
                }
            }
        }

        ray.org = ray.org + t * ray.dir;
        current_path_throughput *= (transmittance / trans_pdf);
        multi_trans_pdf *= trans_pdf;

        // If we hit a surface (didn't scatter), include emission with MIS
        if (!scatter && vertex_) {
            if (is_light(scene.shapes[vertex_->shape_id])) {
                if (never_scatter) {
                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene);
                } else {
                    int light_id = get_area_light_id(scene.shapes[vertex_->shape_id]);
                    const Light &light = scene.lights[light_id];
                    PointAndNormal light_point{vertex_->position, vertex_->geometric_normal};

                    Real pdf_nee = light_pmf(scene, light_id) *
                                   pdf_point_on_light(light, light_point, nee_p_cache, scene);

                    Real dist_sq = distance_squared(nee_p_cache, vertex_->position);
                    Real G = max(dot(-ray.dir, vertex_->geometric_normal), Real(0)) / dist_sq;

                    Real pdf_dir = dir_pdf * multi_trans_pdf * G;
                    Real w = (pdf_dir * pdf_dir) / (pdf_dir * pdf_dir + pdf_nee * pdf_nee);

                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene) * w;
                }
            }
        }

        if (!scatter && !vertex_) {
            break;
        }

        if (max_depth != -1 && bounces >= max_depth - 1) {
            break;
        }

        // Handle index-matching surfaces
        if (!scatter && vertex_) {
            if (vertex_->material_id == -1) {
                current_medium_id = update_medium(ray, *vertex_, current_medium_id);
                ray.org = ray.org + get_intersection_epsilon(scene) * ray.dir;
                bounces++;
                continue;
            }
            // Opaque surface - handle surface lighting below
        }

        // Handle scattering (volume or surface)
        if (scatter) {
            // Volume scattering
            const Medium &medium = scene.media[current_medium_id];
            PhaseFunction phase_func = get_phase_function(medium);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);

            // ============ Next Event Estimation for volume ============
            Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real light_w = next_pcg32_real<Real>(rng);
            Real shape_w = next_pcg32_real<Real>(rng);
            int light_id = sample_light(scene, light_w);
            const Light &light = scene.lights[light_id];
            PointAndNormal point_on_light = sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

            Vector3 dir_light = normalize(point_on_light.position - ray.org);
            Real full_dist = distance(point_on_light.position, ray.org);

            // Trace shadow ray through index-matching surfaces
            Spectrum T_light = make_const_spectrum(1);
            Real p_trans_dir = 1;
            int shadow_medium_id = current_medium_id;
            Vector3 shadow_pos = ray.org;
            int shadow_bounces = 0;
            bool blocked = false;

            while (true) {
                Ray shadow_ray{shadow_pos, dir_light, get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) * distance(shadow_pos, point_on_light.position)};
                std::optional<PathVertex> shadow_isect = intersect(scene, shadow_ray, ray_diff);

                Real next_t = distance(shadow_pos, point_on_light.position);
                if (shadow_isect) {
                    next_t = distance(shadow_pos, shadow_isect->position);
                }

                if (shadow_medium_id >= 0) {
                    const Medium &shadow_medium = scene.media[shadow_medium_id];
                    Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_pos) +
                                              get_sigma_s(shadow_medium, shadow_pos);
                    Real shadow_sigma_t_scalar = shadow_sigma_t[0];
                    T_light *= exp(-shadow_sigma_t * next_t);
                    p_trans_dir *= exp(-shadow_sigma_t_scalar * next_t);
                }

                if (!shadow_isect) {
                    break;
                } else {
                    if (shadow_isect->material_id >= 0) {
                        blocked = true;
                        break;
                    }
                    shadow_bounces++;
                    if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
                        blocked = true;
                        break;
                    }
                    shadow_medium_id = update_medium(
                        Ray{shadow_pos, dir_light}, *shadow_isect, shadow_medium_id);
                    shadow_pos = shadow_isect->position + get_intersection_epsilon(scene) * dir_light;
                }
            }

            if (!blocked && max(T_light) > 0) {
                Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                         (full_dist * full_dist);
                Spectrum phase_val = eval(phase_func, -ray.dir, dir_light);
                Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

                Real pdf_nee = light_pmf(scene, light_id) *
                               pdf_point_on_light(light, point_on_light, ray.org, scene);

                if (pdf_nee > 0) {
                    Spectrum contrib = T_light * G * phase_val * Le / pdf_nee;
                    Real pdf_phase = pdf_sample_phase(phase_func, -ray.dir, dir_light) * G * p_trans_dir;
                    Real w = (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);
                    radiance += current_path_throughput * sigma_s * contrib * w;
                }
            }

            // ============ Phase Function Sampling ============
            Vector2 phase_rnd{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            std::optional<Vector3> next_dir_ = sample_phase_function(phase_func, -ray.dir, phase_rnd);

            if (!next_dir_) {
                break;
            }

            Vector3 next_dir = *next_dir_;
            Spectrum phase_val = eval(phase_func, -ray.dir, next_dir);
            Real phase_pdf = pdf_sample_phase(phase_func, -ray.dir, next_dir);

            if (phase_pdf <= 0) {
                break;
            }

            dir_pdf = phase_pdf;
            nee_p_cache = ray.org;
            multi_trans_pdf = 1;
            never_scatter = false;

            current_path_throughput *= (phase_val / phase_pdf) * sigma_s;
            ray.dir = next_dir;
        } else if (vertex_ && vertex_->material_id >= 0) {
            // Surface scattering
            const Material &mat = scene.materials[vertex_->material_id];
            Vector3 dir_view = -ray.dir;

            // ============ Next Event Estimation for surface ============
            Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real light_w = next_pcg32_real<Real>(rng);
            Real shape_w = next_pcg32_real<Real>(rng);
            int light_id = sample_light(scene, light_w);
            const Light &light = scene.lights[light_id];
            PointAndNormal point_on_light = sample_point_on_light(light, vertex_->position, light_uv, shape_w, scene);

            Vector3 dir_light = normalize(point_on_light.position - vertex_->position);
            Real full_dist = distance(point_on_light.position, vertex_->position);

            // Trace shadow ray through volumes and index-matching surfaces
            Spectrum T_light = make_const_spectrum(1);
            Real p_trans_dir = 1;
            int shadow_medium_id = update_medium(ray, *vertex_, current_medium_id);
            // Check if we're on the correct side
            if (dot(dir_light, vertex_->geometric_normal) * dot(dir_view, vertex_->geometric_normal) > 0) {
                shadow_medium_id = current_medium_id;
            }
            Vector3 shadow_pos = vertex_->position;
            int shadow_bounces = 0;
            bool blocked = false;

            while (true) {
                Ray shadow_ray{shadow_pos, dir_light, get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) * distance(shadow_pos, point_on_light.position)};
                std::optional<PathVertex> shadow_isect = intersect(scene, shadow_ray, ray_diff);

                Real next_t = distance(shadow_pos, point_on_light.position);
                if (shadow_isect) {
                    next_t = distance(shadow_pos, shadow_isect->position);
                }

                if (shadow_medium_id >= 0) {
                    const Medium &shadow_medium = scene.media[shadow_medium_id];
                    Spectrum shadow_sigma_t = get_sigma_a(shadow_medium, shadow_pos) +
                                              get_sigma_s(shadow_medium, shadow_pos);
                    Real shadow_sigma_t_scalar = shadow_sigma_t[0];
                    T_light *= exp(-shadow_sigma_t * next_t);
                    p_trans_dir *= exp(-shadow_sigma_t_scalar * next_t);
                }

                if (!shadow_isect) {
                    break;
                } else {
                    if (shadow_isect->material_id >= 0) {
                        blocked = true;
                        break;
                    }
                    shadow_bounces++;
                    if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
                        blocked = true;
                        break;
                    }
                    shadow_medium_id = update_medium(
                        Ray{shadow_pos, dir_light}, *shadow_isect, shadow_medium_id);
                    shadow_pos = shadow_isect->position + get_intersection_epsilon(scene) * dir_light;
                }
            }

            if (!blocked && max(T_light) > 0) {
                Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                         (full_dist * full_dist);
                Spectrum f = eval(mat, dir_view, dir_light, *vertex_, scene.texture_pool);
                Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

                Real pdf_nee = light_pmf(scene, light_id) *
                               pdf_point_on_light(light, point_on_light, vertex_->position, scene);

                if (pdf_nee > 0) {
                    Spectrum contrib = T_light * G * f * Le / pdf_nee;
                    Real pdf_bsdf = pdf_sample_bsdf(mat, dir_view, dir_light, *vertex_, scene.texture_pool) * G * p_trans_dir;
                    Real w = (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_bsdf * pdf_bsdf);
                    radiance += current_path_throughput * contrib * w;
                }
            }

            // ============ BSDF Sampling ============
            Vector2 bsdf_rnd_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real bsdf_rnd_w = next_pcg32_real<Real>(rng);
            std::optional<BSDFSampleRecord> bsdf_sample_ =
                sample_bsdf(mat, dir_view, *vertex_, scene.texture_pool, bsdf_rnd_uv, bsdf_rnd_w);

            if (!bsdf_sample_) {
                break;
            }

            Vector3 dir_bsdf = bsdf_sample_->dir_out;
            Spectrum f = eval(mat, dir_view, dir_bsdf, *vertex_, scene.texture_pool);
            Real bsdf_pdf = pdf_sample_bsdf(mat, dir_view, dir_bsdf, *vertex_, scene.texture_pool);

            if (bsdf_pdf <= 0) {
                break;
            }

            dir_pdf = bsdf_pdf;
            nee_p_cache = vertex_->position;
            multi_trans_pdf = 1;
            never_scatter = false;

            current_path_throughput *= f / bsdf_pdf;

            // Update medium based on transmission
            if (bsdf_sample_->eta != 0) {
                // Transmission event
                current_medium_id = update_medium(Ray{vertex_->position, dir_bsdf}, *vertex_, current_medium_id);
            }

            ray.org = vertex_->position + get_intersection_epsilon(scene) * dir_bsdf;
            ray.dir = dir_bsdf;
        }

        // Russian roulette
        Real rr_prob = 1;
        if (bounces >= rr_depth) {
            rr_prob = min(max(current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                break;
            }
            current_path_throughput /= rr_prob;
        }

        bounces++;
    }

    return radiance;
}

// The final volumetric renderer:
// multiple chromatic heterogeneous volumes with multiple scattering
// with MIS between next event estimation and phase function sampling
// with surface lighting
Spectrum vol_path_tracing(const Scene &scene,
                          int x, int y, /* pixel coordinates */
                          pcg32_state &rng) {
    // Generate camera ray
    int w = scene.camera.width, h = scene.camera.height;
    Vector2 screen_pos((x + next_pcg32_real<Real>(rng)) / w,
                       (y + next_pcg32_real<Real>(rng)) / h);
    Ray ray = sample_primary(scene.camera, screen_pos);
    RayDifferential ray_diff = RayDifferential{Real(0), Real(0)};

    int current_medium_id = scene.camera.medium_id;
    Spectrum current_path_throughput = make_const_spectrum(1);
    Spectrum radiance = make_zero_spectrum();
    int bounces = 0;
    int max_depth = scene.options.max_depth;
    int rr_depth = scene.options.rr_depth;
    int max_null_collisions = scene.options.max_null_collisions;

    // For MIS: cache quantities for computing PDF of alternative sampling (now Spectrum for chromatic)
    Real dir_pdf = 0;              // PDF of directional sampling (solid angle)
    Vector3 nee_p_cache = ray.org; // Position that can issue NEE
    Spectrum multi_trans_pdf = make_const_spectrum(1); // Product of transmittance PDFs (Spectrum for chromatic)
    bool never_scatter = true;

    while (true) {
        bool scatter = false;

        // Intersect with scene
        std::optional<PathVertex> vertex_ = intersect(scene, ray, ray_diff);
        Real t_hit = infinity<Real>();
        if (vertex_) {
            t_hit = distance(ray.org, vertex_->position);
        }

        Spectrum transmittance = make_const_spectrum(1);
        Spectrum trans_dir_pdf = make_const_spectrum(1);  // PDF for free-flight sampling
        Spectrum trans_nee_pdf = make_const_spectrum(1);  // PDF for next event estimation
        Real t = t_hit;

        if (current_medium_id >= 0) {
            const Medium &medium = scene.media[current_medium_id];
            Spectrum majorant = get_majorant(medium, ray);

            // Sample a channel for sampling
            int channel = std::clamp(int(next_pcg32_real<Real>(rng) * 3), 0, 2);

            Real accum_t = 0;
            int iteration = 0;

            while (majorant[channel] > 0 && iteration < max_null_collisions) {
                Real t_sample = -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
                Real dt = t_hit - accum_t;
                accum_t = min(accum_t + t_sample, t_hit);

                if (t_sample < dt) {
                    // Haven't reached the surface - check real/fake particle
                    Vector3 pos = ray.org + accum_t * ray.dir;
                    Spectrum sigma_a = get_sigma_a(medium, pos);
                    Spectrum sigma_s = get_sigma_s(medium, pos);
                    Spectrum sigma_t = sigma_a + sigma_s;
                    Spectrum sigma_n = majorant - sigma_t;

                    // Clamp sigma_n to avoid negative values due to numerical errors
                    sigma_n = max(sigma_n, make_zero_spectrum());

                    Spectrum real_prob = sigma_t / majorant;

                    if (next_pcg32_real<Real>(rng) < real_prob[channel]) {
                        // Hit a "real" particle - scatter
                        scatter = true;
                        t = accum_t;

                        Real max_maj = max(majorant);
                        transmittance *= exp(-majorant * t_sample) / max_maj;
                        trans_dir_pdf *= exp(-majorant * t_sample) * majorant * real_prob / max_maj;
                        // trans_nee_pdf not needed since we scatter
                        break;
                    } else {
                        // Hit a "fake" particle - continue
                        Real max_maj = max(majorant);
                        transmittance *= exp(-majorant * t_sample) * sigma_n / max_maj;
                        trans_dir_pdf *= exp(-majorant * t_sample) * majorant * (make_const_spectrum(1) - real_prob) / max_maj;
                        trans_nee_pdf *= exp(-majorant * t_sample) * majorant / max_maj;
                    }
                } else {
                    // Reached the surface
                    t = t_hit;
                    transmittance *= exp(-majorant * dt);
                    trans_dir_pdf *= exp(-majorant * dt);
                    trans_nee_pdf *= exp(-majorant * dt);
                    break;
                }
                iteration++;
            }
        }

        ray.org = ray.org + t * ray.dir;

        // Compute average PDF for one-sample MIS
        Real avg_trans_dir_pdf = (trans_dir_pdf[0] + trans_dir_pdf[1] + trans_dir_pdf[2]) / 3.0;
        if (avg_trans_dir_pdf > 0) {
            current_path_throughput *= (transmittance / avg_trans_dir_pdf);
        }

        // Update multi_trans_pdf for MIS
        multi_trans_pdf *= trans_dir_pdf;

        // If we hit a surface (didn't scatter), include emission with MIS
        if (!scatter && vertex_) {
            if (is_light(scene.shapes[vertex_->shape_id])) {
                if (never_scatter) {
                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene);
                } else {
                    int light_id = get_area_light_id(scene.shapes[vertex_->shape_id]);
                    const Light &light = scene.lights[light_id];
                    PointAndNormal light_point{vertex_->position, vertex_->geometric_normal};

                    Real pdf_nee = light_pmf(scene, light_id) *
                                   pdf_point_on_light(light, light_point, nee_p_cache, scene);

                    Real dist_sq = distance_squared(nee_p_cache, vertex_->position);
                    Real G = max(dot(-ray.dir, vertex_->geometric_normal), Real(0)) / dist_sq;

                    Real avg_multi_trans_pdf = (multi_trans_pdf[0] + multi_trans_pdf[1] + multi_trans_pdf[2]) / 3.0;
                    Real pdf_dir = dir_pdf * avg_multi_trans_pdf * G;
                    Real w = (pdf_dir * pdf_dir) / (pdf_dir * pdf_dir + pdf_nee * pdf_nee);

                    radiance += current_path_throughput * emission(*vertex_, -ray.dir, scene) * w;
                }
            }
        }

        if (!scatter && !vertex_) {
            break;
        }

        if (max_depth != -1 && bounces >= max_depth - 1) {
            break;
        }

        // Handle index-matching surfaces
        if (!scatter && vertex_) {
            if (vertex_->material_id == -1) {
                current_medium_id = update_medium(ray, *vertex_, current_medium_id);
                ray.org = ray.org + get_intersection_epsilon(scene) * ray.dir;
                bounces++;
                continue;
            }
            // Opaque surface - handle below
        }

        // Handle scattering (volume or surface)
        if (scatter) {
            // Volume scattering
            const Medium &medium = scene.media[current_medium_id];
            PhaseFunction phase_func = get_phase_function(medium);
            Spectrum sigma_s = get_sigma_s(medium, ray.org);

            // ============ Next Event Estimation using ratio tracking ============
            Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real light_w = next_pcg32_real<Real>(rng);
            Real shape_w = next_pcg32_real<Real>(rng);
            int light_id = sample_light(scene, light_w);
            const Light &light = scene.lights[light_id];
            PointAndNormal point_on_light = sample_point_on_light(light, ray.org, light_uv, shape_w, scene);

            Vector3 dir_light = normalize(point_on_light.position - ray.org);
            Real full_dist = distance(point_on_light.position, ray.org);

            // Compute transmittance to light using ratio tracking
            Spectrum T_light = make_const_spectrum(1);
            Spectrum p_trans_nee = make_const_spectrum(1);
            Spectrum p_trans_dir = make_const_spectrum(1);
            int shadow_medium_id = current_medium_id;
            Vector3 shadow_pos = ray.org;
            int shadow_bounces = 0;
            bool blocked = false;

            while (true) {
                Ray shadow_ray{shadow_pos, dir_light, get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) * distance(shadow_pos, point_on_light.position)};
                std::optional<PathVertex> shadow_isect = intersect(scene, shadow_ray, ray_diff);

                Real next_t = distance(shadow_pos, point_on_light.position);
                if (shadow_isect) {
                    next_t = distance(shadow_pos, shadow_isect->position);
                }

                // Ratio tracking for transmittance estimation
                if (shadow_medium_id >= 0) {
                    const Medium &shadow_medium = scene.media[shadow_medium_id];
                    Spectrum majorant = get_majorant(shadow_medium, shadow_ray);

                    int channel = std::clamp(int(next_pcg32_real<Real>(rng) * 3), 0, 2);
                    Real accum_t = 0;
                    int iteration = 0;

                    while (majorant[channel] > 0 && iteration < max_null_collisions) {
                        Real t_sample = -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
                        Real dt = next_t - accum_t;
                        accum_t = min(accum_t + t_sample, next_t);

                        if (t_sample < dt) {
                            // Null scattering event
                            Vector3 pos = shadow_pos + accum_t * dir_light;
                            Spectrum sigma_t_local = get_sigma_a(shadow_medium, pos) +
                                                     get_sigma_s(shadow_medium, pos);
                            Spectrum sigma_n = majorant - sigma_t_local;
                            sigma_n = max(sigma_n, make_zero_spectrum());

                            Real max_maj = max(majorant);
                            T_light *= exp(-majorant * t_sample) * sigma_n / max_maj;
                            p_trans_nee *= exp(-majorant * t_sample) * majorant / max_maj;

                            Spectrum real_prob = sigma_t_local / majorant;
                            p_trans_dir *= exp(-majorant * t_sample) * majorant * (make_const_spectrum(1) - real_prob) / max_maj;

                            if (max(T_light) <= 0) {
                                break;
                            }
                        } else {
                            // Reached next surface
                            T_light *= exp(-majorant * dt);
                            p_trans_nee *= exp(-majorant * dt);
                            p_trans_dir *= exp(-majorant * dt);
                            break;
                        }
                        iteration++;
                    }
                }

                if (!shadow_isect) {
                    break;
                } else {
                    if (shadow_isect->material_id >= 0) {
                        blocked = true;
                        break;
                    }
                    shadow_bounces++;
                    if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
                        blocked = true;
                        break;
                    }
                    shadow_medium_id = update_medium(
                        Ray{shadow_pos, dir_light}, *shadow_isect, shadow_medium_id);
                    shadow_pos = shadow_isect->position + get_intersection_epsilon(scene) * dir_light;
                }
            }

            if (!blocked && max(T_light) > 0) {
                Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                         (full_dist * full_dist);
                Spectrum phase_val = eval(phase_func, -ray.dir, dir_light);
                Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

                Real pdf_nee = light_pmf(scene, light_id) *
                               pdf_point_on_light(light, point_on_light, ray.org, scene);

                if (pdf_nee > 0) {
                    Real avg_p_trans_nee = (p_trans_nee[0] + p_trans_nee[1] + p_trans_nee[2]) / 3.0;
                    Spectrum contrib = (T_light / avg_p_trans_nee) * G * phase_val * Le / pdf_nee;

                    Real avg_p_trans_dir = (p_trans_dir[0] + p_trans_dir[1] + p_trans_dir[2]) / 3.0;
                    Real pdf_phase = pdf_sample_phase(phase_func, -ray.dir, dir_light) * G * avg_p_trans_dir;
                    Real w = (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_phase * pdf_phase);
                    radiance += current_path_throughput * sigma_s * contrib * w;
                }
            }

            // ============ Phase Function Sampling ============
            Vector2 phase_rnd{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            std::optional<Vector3> next_dir_ = sample_phase_function(phase_func, -ray.dir, phase_rnd);

            if (!next_dir_) {
                break;
            }

            Vector3 next_dir = *next_dir_;
            Spectrum phase_val = eval(phase_func, -ray.dir, next_dir);
            Real phase_pdf = pdf_sample_phase(phase_func, -ray.dir, next_dir);

            if (phase_pdf <= 0) {
                break;
            }

            dir_pdf = phase_pdf;
            nee_p_cache = ray.org;
            multi_trans_pdf = make_const_spectrum(1);
            never_scatter = false;

            current_path_throughput *= (phase_val / phase_pdf) * sigma_s;
            ray.dir = next_dir;
        } else if (vertex_ && vertex_->material_id >= 0) {
            // Surface scattering
            const Material &mat = scene.materials[vertex_->material_id];
            Vector3 dir_view = -ray.dir;

            // ============ Next Event Estimation for surface ============
            Vector2 light_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real light_w = next_pcg32_real<Real>(rng);
            Real shape_w = next_pcg32_real<Real>(rng);
            int light_id = sample_light(scene, light_w);
            const Light &light = scene.lights[light_id];
            PointAndNormal point_on_light = sample_point_on_light(light, vertex_->position, light_uv, shape_w, scene);

            Vector3 dir_light = normalize(point_on_light.position - vertex_->position);
            Real full_dist = distance(point_on_light.position, vertex_->position);

            // Ratio tracking for transmittance to light
            Spectrum T_light = make_const_spectrum(1);
            Spectrum p_trans_nee = make_const_spectrum(1);
            Spectrum p_trans_dir = make_const_spectrum(1);
            int shadow_medium_id = update_medium(ray, *vertex_, current_medium_id);
            if (dot(dir_light, vertex_->geometric_normal) * dot(dir_view, vertex_->geometric_normal) > 0) {
                shadow_medium_id = current_medium_id;
            }
            Vector3 shadow_pos = vertex_->position;
            int shadow_bounces = 0;
            bool blocked = false;

            while (true) {
                Ray shadow_ray{shadow_pos, dir_light, get_shadow_epsilon(scene),
                               (1 - get_shadow_epsilon(scene)) * distance(shadow_pos, point_on_light.position)};
                std::optional<PathVertex> shadow_isect = intersect(scene, shadow_ray, ray_diff);

                Real next_t = distance(shadow_pos, point_on_light.position);
                if (shadow_isect) {
                    next_t = distance(shadow_pos, shadow_isect->position);
                }

                if (shadow_medium_id >= 0) {
                    const Medium &shadow_medium = scene.media[shadow_medium_id];
                    Spectrum majorant = get_majorant(shadow_medium, shadow_ray);

                    int channel = std::clamp(int(next_pcg32_real<Real>(rng) * 3), 0, 2);
                    Real accum_t = 0;
                    int iteration = 0;

                    while (majorant[channel] > 0 && iteration < max_null_collisions) {
                        Real t_sample = -log(1 - next_pcg32_real<Real>(rng)) / majorant[channel];
                        Real dt = next_t - accum_t;
                        accum_t = min(accum_t + t_sample, next_t);

                        if (t_sample < dt) {
                            Vector3 pos = shadow_pos + accum_t * dir_light;
                            Spectrum sigma_t_local = get_sigma_a(shadow_medium, pos) +
                                                     get_sigma_s(shadow_medium, pos);
                            Spectrum sigma_n = majorant - sigma_t_local;
                            sigma_n = max(sigma_n, make_zero_spectrum());

                            Real max_maj = max(majorant);
                            T_light *= exp(-majorant * t_sample) * sigma_n / max_maj;
                            p_trans_nee *= exp(-majorant * t_sample) * majorant / max_maj;

                            Spectrum real_prob = sigma_t_local / majorant;
                            p_trans_dir *= exp(-majorant * t_sample) * majorant * (make_const_spectrum(1) - real_prob) / max_maj;

                            if (max(T_light) <= 0) {
                                break;
                            }
                        } else {
                            T_light *= exp(-majorant * dt);
                            p_trans_nee *= exp(-majorant * dt);
                            p_trans_dir *= exp(-majorant * dt);
                            break;
                        }
                        iteration++;
                    }
                }

                if (!shadow_isect) {
                    break;
                } else {
                    if (shadow_isect->material_id >= 0) {
                        blocked = true;
                        break;
                    }
                    shadow_bounces++;
                    if (max_depth != -1 && bounces + shadow_bounces + 1 >= max_depth) {
                        blocked = true;
                        break;
                    }
                    shadow_medium_id = update_medium(
                        Ray{shadow_pos, dir_light}, *shadow_isect, shadow_medium_id);
                    shadow_pos = shadow_isect->position + get_intersection_epsilon(scene) * dir_light;
                }
            }

            if (!blocked && max(T_light) > 0) {
                Real G = max(-dot(dir_light, point_on_light.normal), Real(0)) /
                         (full_dist * full_dist);
                Spectrum f = eval(mat, dir_view, dir_light, *vertex_, scene.texture_pool);
                Spectrum Le = emission(light, -dir_light, Real(0), point_on_light, scene);

                Real pdf_nee = light_pmf(scene, light_id) *
                               pdf_point_on_light(light, point_on_light, vertex_->position, scene);

                if (pdf_nee > 0) {
                    Real avg_p_trans_nee = (p_trans_nee[0] + p_trans_nee[1] + p_trans_nee[2]) / 3.0;
                    Spectrum contrib = (T_light / avg_p_trans_nee) * G * f * Le / pdf_nee;

                    Real avg_p_trans_dir = (p_trans_dir[0] + p_trans_dir[1] + p_trans_dir[2]) / 3.0;
                    Real pdf_bsdf = pdf_sample_bsdf(mat, dir_view, dir_light, *vertex_, scene.texture_pool) * G * avg_p_trans_dir;
                    Real w = (pdf_nee * pdf_nee) / (pdf_nee * pdf_nee + pdf_bsdf * pdf_bsdf);
                    radiance += current_path_throughput * contrib * w;
                }
            }

            // ============ BSDF Sampling ============
            Vector2 bsdf_rnd_uv{next_pcg32_real<Real>(rng), next_pcg32_real<Real>(rng)};
            Real bsdf_rnd_w = next_pcg32_real<Real>(rng);
            std::optional<BSDFSampleRecord> bsdf_sample_ =
                sample_bsdf(mat, dir_view, *vertex_, scene.texture_pool, bsdf_rnd_uv, bsdf_rnd_w);

            if (!bsdf_sample_) {
                break;
            }

            Vector3 dir_bsdf = bsdf_sample_->dir_out;
            Spectrum f = eval(mat, dir_view, dir_bsdf, *vertex_, scene.texture_pool);
            Real bsdf_pdf = pdf_sample_bsdf(mat, dir_view, dir_bsdf, *vertex_, scene.texture_pool);

            if (bsdf_pdf <= 0) {
                break;
            }

            dir_pdf = bsdf_pdf;
            nee_p_cache = vertex_->position;
            multi_trans_pdf = make_const_spectrum(1);
            never_scatter = false;

            current_path_throughput *= f / bsdf_pdf;

            if (bsdf_sample_->eta != 0) {
                current_medium_id = update_medium(Ray{vertex_->position, dir_bsdf}, *vertex_, current_medium_id);
            }

            ray.org = vertex_->position + get_intersection_epsilon(scene) * dir_bsdf;
            ray.dir = dir_bsdf;
        }

        // Russian roulette
        Real rr_prob = 1;
        if (bounces >= rr_depth) {
            rr_prob = min(max(current_path_throughput), Real(0.95));
            if (next_pcg32_real<Real>(rng) > rr_prob) {
                break;
            }
            current_path_throughput /= rr_prob;
        }

        bounces++;
    }

    return radiance;
}
