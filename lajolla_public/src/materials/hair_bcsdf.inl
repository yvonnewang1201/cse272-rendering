// Hair BCSDF implementation based on:
// - Marschner et al., "Light scattering from human hair fibers" (SIGGRAPH 2003)
// - d'Eon et al., "An energy-conserving hair reflectance model" (Eurographics 2011)
// - PBRT v3/v4 hair implementation

#include "../microfacet.h"
#include <cmath>
#include <array>

// =============================================================================
// DEBUG FLAGS - Set to true to enable debug visualization
// =============================================================================
// Debug Task 1: Visualize UV pipeline - outputs tex_uv as RGB color
// If seal shows smooth R/G gradient across body = UV pipeline works
// If all hairs are same color = surface_uv is broken
static const bool DEBUG_UV_AS_COLOR = false;  // Disabled after UV verification

// Debug Task 2: Isolate firefly source by disabling lobes
// Enable ONE of these at a time to find which lobe causes fireflies
static const bool DEBUG_DISABLE_R_LOBE = false;    // Disable primary specular
static const bool DEBUG_DISABLE_TT_LOBE = false;   // Disable transmission
static const bool DEBUG_DISABLE_TRT_LOBE = false;  // Disable internal reflection (most likely culprit)

// Debug: Use ONLY R lobe (simplest, should be firefly-free)
static const bool DEBUG_R_LOBE_ONLY = false;

// =============================================================================
// DEBUG TASK 3: ALBEDO ONLY MODE (v53)
// =============================================================================
// Bypass ALL Hair BCSDF scattering math and return raw texture color.
// This shows what the texture ACTUALLY looks like on the fur geometry.
// If spots are visible here but not in normal render → BSDF math is broken
// If spots are NOT visible here → texture/UV pipeline is broken
// =============================================================================
static const bool DEBUG_ALBEDO_ONLY = false;  // DISABLED - diagnostic complete
// =============================================================================

// =============================================================================
// v59 FIX: DIFFUSE FALLBACK FOR MULTIPLE SCATTERING
// =============================================================================
// Previous attempts (v54-v58) failed because:
// - Boosting scales > 1.0 violates energy conservation
// - Boosting TT causes blue HDRI to transmit through (glass effect)
// - sigma_a near zero makes fur transparent
//
// Root cause: Single-scattering Hair BCSDF loses energy in dense fur.
// Real fur has multiple scattering that redistributes light inside the pelt.
//
// v59 Fix: Add a DIFFUSE FALLBACK term to represent multiple-scattered light.
// This is bounded, energy-conserving, and uses the reflectance texture directly.
// No more >1.0 scale hacks or transparency tricks.
// =============================================================================
static const bool USE_DIFFUSE_FALLBACK = true;   // v59: Add diffuse for dense fur
static const Real DIFFUSE_WEIGHT = Real(0.60);   // v59b: Increased from 0.35
static const bool KILL_TT_LOBE = true;           // v59: No transmission in dense fur
// =============================================================================

// Hair coordinate system:
// For hair, we use a different coordinate system than regular surfaces.
// The shading frame's tangent (x-axis) represents the hair fiber direction.
// The normal (z-axis) points outward from the hair cylinder.
// theta_i, theta_o: angles from the normal plane (longitude)
// phi_i, phi_o: azimuthal angles around the hair

namespace hair {

// Constants
static const Real kMinCosTheta = Real(1e-4);  // Minimum cos_theta to avoid division by zero

// Simple hash for tangent jitter (based on UV coordinates)
inline Real hash_uv(Real u, Real v, Real seed) {
    // Simple pseudo-random based on UV coordinates
    Real x = sin(u * 12.9898 + v * 78.233 + seed) * 43758.5453;
    return x - floor(x);
}

// Apply tangent jitter to frame for micro-fiber scattering effect
// jitter_amount: 0.0 = no jitter, 0.1 = subtle, 0.3 = strong
inline Frame apply_tangent_jitter(const Frame &frame, const Vector2 &uv, Real jitter_amount) {
    if (jitter_amount <= Real(0)) return frame;

    // Generate random angles based on UV for stable micro-variation
    Real angle_x = (hash_uv(uv.x * 100, uv.y * 100, Real(0)) - Real(0.5)) * jitter_amount;
    Real angle_y = (hash_uv(uv.x * 100, uv.y * 100, Real(1)) - Real(0.5)) * jitter_amount;

    // Perturb normal slightly
    Vector3 new_n = normalize(frame.n + frame.x * sin(angle_x) + frame.y * sin(angle_y));

    // Recompute orthonormal frame
    Vector3 new_x = normalize(frame.x - new_n * dot(frame.x, new_n));
    Vector3 new_y = cross(new_n, new_x);

    return Frame{new_x, new_y, new_n};
}

// Compute sin(theta) and cos(theta) from direction in hair local frame
inline void compute_hair_angles(const Vector3 &w, Real &sin_theta, Real &cos_theta, Real &phi) {
    // In hair local frame: x = tangent (along hair), y and z span the normal plane
    // theta is the angle from the normal plane
    sin_theta = std::clamp(w.x, Real(-1), Real(1));  // component along the hair
    cos_theta = sqrt(max(Real(0), 1 - sin_theta * sin_theta));
    cos_theta = max(cos_theta, kMinCosTheta);  // Avoid division by zero
    phi = atan2(w.z, w.y);
}

// Compute the angle difference for azimuthal scattering
inline Real compute_phi(Real phi_i, Real phi_o) {
    Real phi = phi_o - phi_i;
    while (phi > c_PI) phi -= c_TWOPI;
    while (phi < -c_PI) phi += c_TWOPI;
    return phi;
}

// Safe I0 (modified Bessel function of first kind) approximation
inline Real safe_I0(Real x) {
    // For numerical stability, clamp the input
    x = min(fabs(x), Real(10));  // Clamp to avoid overflow

    // Polynomial approximation for I0
    if (x < Real(3.75)) {
        Real t = x / Real(3.75);
        Real t2 = t * t;
        return 1 + t2 * (3.5156229 + t2 * (3.0899424 + t2 * (1.2067492 +
               t2 * (0.2659732 + t2 * (0.0360768 + t2 * 0.0045813)))));
    } else {
        Real t = Real(3.75) / x;
        return exp(x) / sqrt(x) * (0.39894228 + t * (0.01328592 +
               t * (0.00225319 + t * (-0.00157565 + t * (0.00916281 +
               t * (-0.02057706 + t * (0.02635537 + t * (-0.01647633 +
               t * 0.00392377))))))));
    }
}

// Longitudinal scattering function M using simple Gaussian
// More numerically stable than the Bessel function version
inline Real M_longitudinal(Real sin_theta_i, Real cos_theta_i,
                           Real sin_theta_o, Real cos_theta_o, Real v) {
    // Clamp variance to avoid numerical issues
    v = max(v, Real(0.001));

    // Simple wrapped Gaussian approach (more stable)
    // theta_d = theta_i + theta_o (difference from specular reflection)
    Real theta_i = asin(std::clamp(sin_theta_i, Real(-1), Real(1)));
    Real theta_o = asin(std::clamp(sin_theta_o, Real(-1), Real(1)));
    Real theta_d = theta_o + theta_i;  // For reflection, specular is when theta_o = -theta_i, so theta_d = 0

    Real sigma = sqrt(v);
    Real M = exp(-theta_d * theta_d / (2 * v)) / (sigma * sqrt(c_TWOPI));

    // Clamp to reasonable range
    return std::clamp(M, Real(0), Real(100));
}

// Fresnel for dielectric at a given cos_theta
inline Real fresnel_hair(Real cos_theta_i, Real eta) {
    cos_theta_i = fabs(cos_theta_i);
    Real sin_theta_i_sq = 1 - cos_theta_i * cos_theta_i;
    Real sin_theta_t_sq = sin_theta_i_sq / (eta * eta);

    if (sin_theta_t_sq >= 1) {
        return 1;  // Total internal reflection
    }

    Real cos_theta_t = sqrt(max(Real(0), 1 - sin_theta_t_sq));
    return fresnel_dielectric(cos_theta_i, cos_theta_t, eta);
}

// Azimuthal scattering function N
// This is a logistic distribution approximation as in d'Eon et al.
inline Real logistic(Real x, Real s) {
    s = max(s, Real(0.01));  // Avoid division by zero
    Real abs_x = fabs(x);
    // Avoid overflow in exp
    if (abs_x / s > 20) return Real(0);
    Real exp_neg = exp(-abs_x / s);
    return exp_neg / (s * (1 + exp_neg) * (1 + exp_neg));
}

inline Real logistic_cdf(Real x, Real s) {
    s = max(s, Real(0.01));
    // Avoid overflow
    if (x / s < -20) return Real(0);
    if (x / s > 20) return Real(1);
    return 1 / (1 + exp(-x / s));
}

inline Real trimmed_logistic(Real x, Real s, Real a, Real b) {
    Real denom = logistic_cdf(b, s) - logistic_cdf(a, s);
    if (denom < Real(1e-6)) return Real(1) / (b - a);  // Uniform fallback
    return logistic(x, s) / denom;
}

// Sample the trimmed logistic distribution
inline Real sample_trimmed_logistic(Real u, Real s, Real a, Real b) {
    s = max(s, Real(0.01));
    Real k = logistic_cdf(b, s) - logistic_cdf(a, s);
    if (k < Real(1e-6)) {
        // Fallback to uniform
        return a + u * (b - a);
    }
    Real arg = u * k + logistic_cdf(a, s);
    arg = std::clamp(arg, Real(1e-6), Real(1 - 1e-6));
    Real x = -s * log(1 / arg - 1);
    return std::clamp(x, a, b);
}

// Compute phi for p-th lobe (exit azimuth after p internal reflections)
inline Real compute_phi_p(int p, Real gamma_o, Real gamma_t) {
    return 2 * p * gamma_t - 2 * gamma_o + p * c_PI;
}

// Absorption through hair fiber with safety checks
inline Spectrum absorption_through_fiber(const Spectrum &sigma_a, Real cos_theta, Real gamma_t, int p) {
    // Safety: clamp cos_theta away from zero
    cos_theta = max(fabs(cos_theta), kMinCosTheta);

    // Path length through fiber
    Real cos_gamma_t = cos(gamma_t);
    Real path_length = 2 * fabs(cos_gamma_t) / cos_theta;
    path_length = min(path_length * p, Real(20));  // Clamp to avoid extreme absorption

    Spectrum T;
    for (int i = 0; i < 3; ++i) {
        T[i] = exp(-sigma_a[i] * path_length);
    }
    return T;
}

// Compute gamma_o and gamma_t from h
inline void compute_gammas(Real h, Real eta, Real &gamma_o, Real &gamma_t) {
    h = std::clamp(h, Real(-0.9999), Real(0.9999));
    gamma_o = asin(h);
    // Snell's law for refraction into the fiber
    Real sin_gamma_t = h / eta;
    sin_gamma_t = std::clamp(sin_gamma_t, Real(-0.9999), Real(0.9999));
    gamma_t = asin(sin_gamma_t);
}

// Attenuation function Ap for each lobe
inline Spectrum Ap(int p, Real cos_theta_o, Real eta, Real h, const Spectrum &T) {
    Real cos_gamma_o = sqrt(max(Real(0), 1 - h * h));
    Real cos_theta = max(cos_theta_o * cos_gamma_o, kMinCosTheta);

    Real f = fresnel_hair(cos_theta, eta);
    f = std::clamp(f, Real(0), Real(1));

    if (p == 0) {
        // R lobe: single surface reflection
        return Spectrum{f, f, f};
    } else if (p == 1) {
        // TT lobe: transmitted through fiber
        Real t = 1 - f;
        return T * t * t;
    } else if (p == 2) {
        // TRT lobe: one internal reflection
        Real t = 1 - f;
        return T * T * t * t * f;
    } else {
        // Higher order lobes (p >= 3)
        Real t = 1 - f;
        Spectrum result = T * T * T * t * t * f * f;
        for (int i = 0; i < 3; ++i) {
            Real denom = 1 - T[i] * f;
            if (denom > Real(1e-6)) {
                result[i] /= denom;
            }
        }
        return result;
    }
}

} // namespace hair

// Evaluate the Hair BCSDF
Spectrum eval_op::operator()(const HairBCSDF &bsdf) const {
    Frame frame = vertex.shading_frame;

    // Apply tangent jitter for micro-fiber scattering (fluffy appearance)
    // Jitter amount of 0.15 gives subtle variation without breaking coherence
    frame = hair::apply_tangent_jitter(frame, vertex.uv, Real(0.15));

    // Transform directions to local frame
    Vector3 wi = to_local(frame, dir_in);
    Vector3 wo = to_local(frame, dir_out);

    // Compute hair angles
    Real sin_theta_i, cos_theta_i, phi_i;
    Real sin_theta_o, cos_theta_o, phi_o;
    hair::compute_hair_angles(wi, sin_theta_i, cos_theta_i, phi_i);
    hair::compute_hair_angles(wo, sin_theta_o, cos_theta_o, phi_o);

    // =========================================================================
    // v32 FIX: Use surface_uv for texture lookup (the ROOT UV from mesh)
    // vertex.uv is curve parameterization and will give wrong texture coords!
    // =========================================================================
    Vector2 tex_uv = vertex.has_surface_uv ? vertex.surface_uv : vertex.uv;

    // =========================================================================
    // v45 FIX: UV V-FLIP for Blender compatibility
    // =========================================================================
    // Blender exports UVs with V=0 at bottom, but the texture might be stored
    // with V=0 at top. Flip V coordinate if texture appears wrong.
    // =========================================================================
    static const bool FLIP_V_COORDINATE = true;  // Enable for Blender exports
    if (FLIP_V_COORDINATE && vertex.has_surface_uv) {
        tex_uv.y = Real(1) - tex_uv.y;
    }

    // =========================================================================
    // DEBUG TASK 1: Visualize UV pipeline
    // Output tex_uv as RGB color to verify UV mapping is working
    // Expected: Smooth R/G gradient across seal body = UV works
    // Bad: All hairs same color = surface_uv pipeline broken
    // =========================================================================
    if (DEBUG_UV_AS_COLOR) {
        // Return UV as color: R = u, G = v, B = 0
        // Also add blue tint if has_surface_uv is true (to verify the flag)
        Real b = vertex.has_surface_uv ? Real(0.3) : Real(0);
        return Spectrum{tex_uv.x, tex_uv.y, b};
    }

    // =========================================================================
    // DEBUG TASK 3: ALBEDO ONLY MODE (v53)
    // =========================================================================
    // Bypass ALL scattering math. Evaluate the reflectance texture directly
    // and return it as an unlit/emissive color. This isolates whether the
    // texture is being sampled correctly from whether the BSDF math is broken.
    //
    // EXPECTED RESULT:
    // - Silver-gray base with dark spots clearly visible
    // - If spots are NOT visible → texture/UV problem
    // - If spots ARE visible → BSDF scattering math is the culprit
    // =========================================================================
    if (DEBUG_ALBEDO_ONLY) {
        // Evaluate the reflectance texture directly at tex_uv
        Spectrum base_color = eval(bsdf.sigma_a, tex_uv, vertex.uv_screen_size, texture_pool);

        // Debug output on first call
        static bool first_albedo_debug = true;
        if (first_albedo_debug) {
            first_albedo_debug = false;
            std::cerr << "[ALBEDO DEBUG] tex_uv=(" << tex_uv.x << "," << tex_uv.y << ")"
                      << ", base_color=(" << base_color[0] << "," << base_color[1] << "," << base_color[2] << ")"
                      << ", use_reflectance=" << bsdf.use_reflectance
                      << std::endl;
        }

        // Return raw texture color (no scattering, no absorption conversion)
        // Scale by a factor to make it visible under normal lighting
        // (since we're bypassing BSDF, cosine term won't multiply correctly)
        return base_color;
    }

    // Get material parameters - use tex_uv for spotted fur pattern
    Spectrum sigma_a = eval(bsdf.sigma_a, tex_uv, vertex.uv_screen_size, texture_pool);
    Real beta_m = eval(bsdf.beta_m, tex_uv, vertex.uv_screen_size, texture_pool);
    Real beta_n = eval(bsdf.beta_n, tex_uv, vertex.uv_screen_size, texture_pool);

    // v45: Debug output to verify texture is being sampled correctly
    static bool first_hair_eval = true;
    if (first_hair_eval) {
        first_hair_eval = false;
        std::cerr << "[Hair BSDF Debug] tex_uv=(" << tex_uv.x << "," << tex_uv.y << ")"
                  << ", use_reflectance=" << bsdf.use_reflectance
                  << ", sigma_a_raw=(" << sigma_a[0] << "," << sigma_a[1] << "," << sigma_a[2] << ")"
                  << std::endl;
    }

    // Convert reflectance to absorption if needed: sigma_a = -log(reflectance) * scale
    // This allows using color textures directly (dark colors = high absorption)
    Real tex_luminance = Real(1);  // Track original texture luminance for spot darkening
    if (bsdf.use_reflectance) {
        // Calculate luminance BEFORE converting to absorption
        tex_luminance = Real(0.2126) * sigma_a[0] + Real(0.7152) * sigma_a[1] + Real(0.0722) * sigma_a[2];
        for (int i = 0; i < 3; ++i) {
            // Clamp to avoid log(0)
            Real refl = std::clamp(sigma_a[i], Real(0.01), Real(0.99));
            sigma_a[i] = -log(refl) * bsdf.sigma_a_scale;
        }
    } else {
        // Apply scale factor to absorption
        sigma_a = sigma_a * bsdf.sigma_a_scale;
    }

    // =========================================================================
    // v35: DARK SPOT SPECULAR ATTENUATION
    // =========================================================================
    // Problem: Black spots appear grey because R-lobe specular overpowers pigment.
    // Solution: Attenuate specular on dark spots to preserve deep blacks.
    // This is physically motivated - dark pigment means more absorption, less
    // surface reflection (melanin absorbs light before it can reflect).
    // =========================================================================
    Real dark_spot_atten = Real(1);
    if (tex_luminance < Real(0.3)) {
        // Linear ramp: lum=0 -> atten=0.3, lum=0.3 -> atten=1.0
        dark_spot_atten = Real(0.3) + (tex_luminance / Real(0.3)) * Real(0.7);
    }

    // =========================================================================
    // v33 PATH SPACE FILTERING: Roughness clamping based on bounce depth
    // =========================================================================
    // Debug results showed: R-only = no fireflies, disable TRT = still fireflies
    // Therefore TT LOBE is the firefly source on indirect bounces.
    //
    // On indirect bounces, widen the TT lobe specifically by using higher
    // roughness for v_TT calculation. This prevents the TT PDF singularity
    // without blurring the primary R-lobe specular.
    // =========================================================================
    Real min_roughness = Real(0.3);  // Base minimum for R lobe
    Real min_roughness_TT = Real(0.3);  // Base for TT lobe
    if (vertex.bounce_depth > 0) {
        // Force MUCH higher roughness for TT on indirect bounces
        // This is the key fix - TT has the tightest PDF (v_TT = v_R/2)
        min_roughness_TT = Real(0.5);  // Aggressively widen TT
    }
    beta_m = max(beta_m, min_roughness);
    beta_n = max(beta_n, min_roughness);

    Real eta = bsdf.eta;
    Real alpha_rad = bsdf.alpha * c_PI / 180;

    // Variance for longitudinal scattering
    Real v_R = beta_m * beta_m;
    // v33: TT uses higher roughness on indirect bounces
    Real beta_m_TT = max(beta_m, min_roughness_TT);
    Real v_TT = beta_m_TT * beta_m_TT / 2;  // Was: v_R / 2
    Real v_TRT = 2 * v_R;

    // Scale for azimuthal scattering (convert roughness to logistic scale)
    Real s = max(beta_n * Real(0.626657), Real(0.1));

    // Compute phi difference
    Real phi = hair::compute_phi(phi_i, phi_o);

    // Use h=0 (ray hits center of fiber)
    Real h = Real(0);
    Real gamma_o, gamma_t;
    hair::compute_gammas(h, eta, gamma_o, gamma_t);

    // Absorption through fiber
    Spectrum T = hair::absorption_through_fiber(sigma_a, cos_theta_o, gamma_t, 1);

    Spectrum fsum = make_zero_spectrum();

    // =========================================================================
    // DEBUG TASK 2: Isolate firefly source by selectively disabling lobes
    // =========================================================================

    // R lobe (p=0) - Primary specular reflection
    // v35: Apply dark_spot_atten to R lobe so black spots stay truly black
    if (!DEBUG_DISABLE_R_LOBE) {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) - 2 * alpha_rad;
        Real sin_theta_i_R = sin(theta_i_shifted);
        Real cos_theta_i_R = cos(theta_i_shifted);

        Real M_R = hair::M_longitudinal(sin_theta_i_R, cos_theta_i_R,
                                        sin_theta_o, cos_theta_o, v_R);
        Real N_R = hair::trimmed_logistic(phi, s, -c_PI, c_PI);
        Spectrum A_R = hair::Ap(0, cos_theta_o, eta, h, T);

        // v35: Attenuate R-lobe on dark spots to preserve deep blacks
        fsum += dark_spot_atten * bsdf.scale_R * A_R * M_R * N_R;
    }

    // If DEBUG_R_LOBE_ONLY is set, skip TT and TRT entirely
    if (!DEBUG_R_LOBE_ONLY) {
        // TT lobe (p=1) - Transmission through fiber
        // v59: KILL TT for dense fur - prevents blue HDRI bleed-through
        if (!DEBUG_DISABLE_TT_LOBE && !KILL_TT_LOBE) {
            Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + alpha_rad;
            Real sin_theta_i_TT = sin(theta_i_shifted);
            Real cos_theta_i_TT = cos(theta_i_shifted);

            Real M_TT = hair::M_longitudinal(sin_theta_i_TT, cos_theta_i_TT,
                                             sin_theta_o, cos_theta_o, v_TT);
            Real phi_TT = hair::compute_phi_p(1, gamma_o, gamma_t);
            Real dphi_TT = hair::compute_phi(phi, phi_TT);
            Real N_TT = hair::trimmed_logistic(dphi_TT, s, -c_PI, c_PI);
            Spectrum A_TT = hair::Ap(1, cos_theta_o, eta, h, T);

            fsum += bsdf.scale_TT * A_TT * M_TT * N_TT;
        }

        // TRT lobe (p=2) - Internal reflection (MOST LIKELY FIREFLY SOURCE)
        if (!DEBUG_DISABLE_TRT_LOBE) {
            Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + 4 * alpha_rad;
            Real sin_theta_i_TRT = sin(theta_i_shifted);
            Real cos_theta_i_TRT = cos(theta_i_shifted);

            Real M_TRT = hair::M_longitudinal(sin_theta_i_TRT, cos_theta_i_TRT,
                                              sin_theta_o, cos_theta_o, v_TRT);
            Real phi_TRT = hair::compute_phi_p(2, gamma_o, gamma_t);
            Real dphi_TRT = hair::compute_phi(phi, phi_TRT);
            Real N_TRT = hair::trimmed_logistic(dphi_TRT, s, -c_PI, c_PI);
            Spectrum A_TRT = hair::Ap(2, cos_theta_o, eta, h, T);

            fsum += bsdf.scale_TRT * A_TRT * M_TRT * N_TRT;
        }
    }

    // =========================================================================
    // v36 NaN/Inf GUARD: Kill mathematical singularities at the source
    // =========================================================================
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(fsum[i]) || std::isinf(fsum[i])) {
            return make_zero_spectrum();
        }
        fsum[i] = std::clamp(fsum[i], Real(0), Real(5));
    }

    // =========================================================================
    // v59 FIX: DIFFUSE FALLBACK FOR MULTIPLE SCATTERING
    // =========================================================================
    // Instead of hacking scales > 1.0 or adding spot modulation, we add a
    // proper diffuse term that represents multiply-scattered light inside
    // the dense fur pelt. This is energy-conserving and physically motivated.
    //
    // The diffuse term uses the REFLECTANCE TEXTURE directly (not sigma_a),
    // so spots appear naturally dark and base fur appears naturally light.
    //
    // Kajiya-Kay inspired diffuse: f_d = Kd * sqrt(1 - sin²θ_i * sin²θ_o)
    // Simplified: use texture color * cos-weighted diffuse approximation
    // =========================================================================
    if (USE_DIFFUSE_FALLBACK) {
        // Get the raw reflectance color (before sigma_a conversion)
        // This is stored in the texture: light gray ~0.63, dark spots ~0.1
        Spectrum reflectance = eval(bsdf.sigma_a, tex_uv, vertex.uv_screen_size, texture_pool);

        // v59b: Simplified Kajiya-Kay diffuse for hair
        // Original K-K: sin(theta_l) * sin(theta_v) but we use cos from normal plane
        // Using abs(cos_theta) to handle back-facing light
        Real diffuse_factor = fabs(cos_theta_i) * fabs(cos_theta_o);

        // Add ambient-like base to prevent total darkness in shadow
        // This represents the multiply-scattered ambient light inside fur
        Real ambient = Real(0.15);
        diffuse_factor = ambient + (Real(1) - ambient) * diffuse_factor;

        // The diffuse contribution: reflectance * diffuse_shading / PI
        Spectrum diffuse = reflectance * diffuse_factor * c_INVPI;

        // Blend: (1 - DIFFUSE_WEIGHT) * specular + DIFFUSE_WEIGHT * diffuse
        // This keeps total energy bounded
        fsum = fsum * (Real(1) - DIFFUSE_WEIGHT) + diffuse * DIFFUSE_WEIGHT;
    }

    return fsum;
}

// PDF for sampling the Hair BCSDF
Real pdf_sample_bsdf_op::operator()(const HairBCSDF &bsdf) const {
    Frame frame = vertex.shading_frame;

    // Apply same tangent jitter as eval for consistency
    frame = hair::apply_tangent_jitter(frame, vertex.uv, Real(0.15));

    Vector3 wi = to_local(frame, dir_in);
    Vector3 wo = to_local(frame, dir_out);

    Real sin_theta_i, cos_theta_i, phi_i;
    Real sin_theta_o, cos_theta_o, phi_o;
    hair::compute_hair_angles(wi, sin_theta_i, cos_theta_i, phi_i);
    hair::compute_hair_angles(wo, sin_theta_o, cos_theta_o, phi_o);

    // v32: Use surface_uv for texture lookup (must match eval!)
    Vector2 tex_uv = vertex.has_surface_uv ? vertex.surface_uv : vertex.uv;
    // v45: Apply V-flip to match eval
    static const bool FLIP_V_COORDINATE = true;
    if (FLIP_V_COORDINATE && vertex.has_surface_uv) {
        tex_uv.y = Real(1) - tex_uv.y;
    }
    Real beta_m = eval(bsdf.beta_m, tex_uv, vertex.uv_screen_size, texture_pool);
    Real beta_n = eval(bsdf.beta_n, tex_uv, vertex.uv_screen_size, texture_pool);

    // v33 PATH SPACE FILTERING: Must match eval exactly!
    // TT lobe uses higher roughness on indirect bounces
    Real min_roughness = Real(0.3);
    Real min_roughness_TT = (vertex.bounce_depth > 0) ? Real(0.5) : Real(0.3);
    beta_m = max(beta_m, min_roughness);
    beta_n = max(beta_n, min_roughness);

    Real eta = bsdf.eta;
    Real alpha_rad = bsdf.alpha * c_PI / 180;

    Real v_R = beta_m * beta_m;
    // v33: TT uses higher roughness on indirect bounces (must match eval!)
    Real beta_m_TT = max(beta_m, min_roughness_TT);
    Real v_TT = beta_m_TT * beta_m_TT / 2;
    Real v_TRT = 2 * v_R;

    Real s = max(beta_n * Real(0.626657), Real(0.1));
    Real phi = hair::compute_phi(phi_i, phi_o);

    Real h = Real(0);
    Real gamma_o, gamma_t;
    hair::compute_gammas(h, eta, gamma_o, gamma_t);

    // Compute lobe weights
    Real weight_R = bsdf.scale_R;
    Real weight_TT = bsdf.scale_TT;
    Real weight_TRT = bsdf.scale_TRT;
    Real total_weight = weight_R + weight_TT + weight_TRT;
    if (total_weight < Real(1e-6)) total_weight = Real(1);

    Real pdf = 0;

    // R lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) - 2 * alpha_rad;
        Real sin_theta_i_R = sin(theta_i_shifted);
        Real cos_theta_i_R = cos(theta_i_shifted);

        Real M_R = hair::M_longitudinal(sin_theta_i_R, cos_theta_i_R,
                                        sin_theta_o, cos_theta_o, v_R);
        Real N_R = hair::trimmed_logistic(phi, s, -c_PI, c_PI);

        pdf += (weight_R / total_weight) * M_R * N_R;
    }

    // TT lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + alpha_rad;
        Real sin_theta_i_TT = sin(theta_i_shifted);
        Real cos_theta_i_TT = cos(theta_i_shifted);

        Real M_TT = hair::M_longitudinal(sin_theta_i_TT, cos_theta_i_TT,
                                         sin_theta_o, cos_theta_o, v_TT);
        Real phi_TT = hair::compute_phi_p(1, gamma_o, gamma_t);
        Real dphi_TT = hair::compute_phi(phi, phi_TT);
        Real N_TT = hair::trimmed_logistic(dphi_TT, s, -c_PI, c_PI);

        pdf += (weight_TT / total_weight) * M_TT * N_TT;
    }

    // TRT lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + 4 * alpha_rad;
        Real sin_theta_i_TRT = sin(theta_i_shifted);
        Real cos_theta_i_TRT = cos(theta_i_shifted);

        Real M_TRT = hair::M_longitudinal(sin_theta_i_TRT, cos_theta_i_TRT,
                                          sin_theta_o, cos_theta_o, v_TRT);
        Real phi_TRT = hair::compute_phi_p(2, gamma_o, gamma_t);
        Real dphi_TRT = hair::compute_phi(phi, phi_TRT);
        Real N_TRT = hair::trimmed_logistic(dphi_TRT, s, -c_PI, c_PI);

        pdf += (weight_TRT / total_weight) * M_TRT * N_TRT;
    }

    // Ensure PDF is positive
    return max(pdf, Real(1e-6));
}

// Sample the Hair BCSDF
std::optional<BSDFSampleRecord> sample_bsdf_op::operator()(const HairBCSDF &bsdf) const {
    Frame frame = vertex.shading_frame;

    // Apply same tangent jitter as eval for consistency
    frame = hair::apply_tangent_jitter(frame, vertex.uv, Real(0.15));

    Vector3 wi = to_local(frame, dir_in);

    Real sin_theta_i, cos_theta_i, phi_i;
    hair::compute_hair_angles(wi, sin_theta_i, cos_theta_i, phi_i);

    // v32: Use surface_uv for texture lookup (must match eval!)
    Vector2 tex_uv = vertex.has_surface_uv ? vertex.surface_uv : vertex.uv;
    // v45: Apply V-flip to match eval
    static const bool FLIP_V_COORDINATE = true;
    if (FLIP_V_COORDINATE && vertex.has_surface_uv) {
        tex_uv.y = Real(1) - tex_uv.y;
    }
    Real beta_m = eval(bsdf.beta_m, tex_uv, vertex.uv_screen_size, texture_pool);
    Real beta_n = eval(bsdf.beta_n, tex_uv, vertex.uv_screen_size, texture_pool);

    // v33 PATH SPACE FILTERING: Must match eval exactly!
    // TT lobe uses higher roughness on indirect bounces
    Real min_roughness = Real(0.3);
    Real min_roughness_TT = (vertex.bounce_depth > 0) ? Real(0.5) : Real(0.3);
    beta_m = max(beta_m, min_roughness);
    beta_n = max(beta_n, min_roughness);

    Real eta = bsdf.eta;
    Real alpha_rad = bsdf.alpha * c_PI / 180;

    Real v_R = beta_m * beta_m;
    // v33: TT uses higher roughness on indirect bounces (must match eval!)
    Real beta_m_TT = max(beta_m, min_roughness_TT);
    Real v_TT = beta_m_TT * beta_m_TT / 2;
    Real v_TRT = 2 * v_R;

    Real s = max(beta_n * Real(0.626657), Real(0.1));

    Real h = Real(0);
    Real gamma_o, gamma_t;
    hair::compute_gammas(h, eta, gamma_o, gamma_t);

    // Choose which lobe to sample
    Real weight_R = bsdf.scale_R;
    Real weight_TT = bsdf.scale_TT;
    Real weight_TRT = bsdf.scale_TRT;
    Real total_weight = weight_R + weight_TT + weight_TRT;
    if (total_weight < Real(1e-6)) total_weight = Real(1);

    Real v_selected;
    Real theta_shift;
    Real phi_lobe;

    Real u = rnd_param_w * total_weight;
    if (u < weight_R) {
        v_selected = v_R;
        theta_shift = -2 * alpha_rad;
        phi_lobe = 0;
    } else if (u < weight_R + weight_TT) {
        v_selected = v_TT;
        theta_shift = alpha_rad;
        phi_lobe = hair::compute_phi_p(1, gamma_o, gamma_t);
    } else {
        v_selected = v_TRT;
        theta_shift = 4 * alpha_rad;
        phi_lobe = hair::compute_phi_p(2, gamma_o, gamma_t);
    }

    // Sample longitudinal angle (theta_o)
    Real theta_i = asin(std::clamp(sin_theta_i, Real(-1), Real(1)));
    Real theta_i_shifted = theta_i + theta_shift;
    Real sigma_m = sqrt(v_selected);

    // Box-Muller for Gaussian sampling
    Real u1 = max(rnd_param_uv[0], Real(1e-6));
    Real u2 = rnd_param_uv[1];
    Real z = sqrt(-2 * log(u1)) * cos(c_TWOPI * u2);

    // Sample theta_o around the reflected angle (theta_o = -theta_i for specular)
    Real theta_o = -theta_i_shifted + sigma_m * z;
    theta_o = std::clamp(theta_o, -c_PI / 2 + Real(0.05), c_PI / 2 - Real(0.05));

    Real sin_theta_o = sin(theta_o);
    Real cos_theta_o = max(cos(theta_o), hair::kMinCosTheta);

    // Sample azimuthal angle (phi_o)
    Real phi_diff = hair::sample_trimmed_logistic(rnd_param_uv[1], s, -c_PI, c_PI);
    Real phi_o = phi_i + phi_diff + phi_lobe;

    // Convert sampled angles back to direction
    Vector3 wo_local{sin_theta_o, cos_theta_o * cos(phi_o), cos_theta_o * sin(phi_o)};

    // Normalize to ensure valid direction
    wo_local = normalize(wo_local);
    Vector3 wo = to_world(frame, wo_local);

    return BSDFSampleRecord{wo, Real(0), beta_m};
}

// Get texture for debugging
TextureSpectrum get_texture_op::operator()(const HairBCSDF &bsdf) const {
    return bsdf.sigma_a;
}
