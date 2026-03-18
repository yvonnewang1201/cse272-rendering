// Oil-coated Hair BCSDF implementation
// Based on position-free Monte Carlo approach (Guo et al. 2018)
// Models wet fur by layering:
// - Top: Smooth dielectric oil film (IOR ~1.5)
// - Substrate: HairBCSDF with modified Fresnel (oil-hair interface)
//
// Key physics: oil IOR (1.5) ≈ hair keratin IOR (1.55)
// This reduces Fresnel reflection at the oil-hair interface,
// making underlying fur "optically invisible" - the wet seal illusion

#include "../microfacet.h"
#include <cmath>

// =============================================================================
// DEBUG FLAGS - Set to true to enable debug visualization
// =============================================================================
// Debug Task 1: Visualize UV pipeline - outputs tex_uv as RGB color
// If seal shows smooth R/G gradient across body = UV pipeline works
// If all hairs are same color = surface_uv is broken
static const bool DEBUG_OIL_UV_AS_COLOR = false;

// Debug Task 2: Disable oil specular to isolate substrate issues
static const bool DEBUG_DISABLE_OIL_SPECULAR = false;

// Debug: Show only oil specular (no substrate) to test GGX
static const bool DEBUG_OIL_SPECULAR_ONLY = false;
// =============================================================================

// Reuse the hair namespace functions from hair_bcsdf.inl
// (They are already defined in the hair namespace)

// Helper: Evaluate hair substrate with modified eta (oil-hair interface)
namespace oil_hair {

// ============================================================================
// Thin-Film Interference
// ============================================================================
// Computes wavelength-dependent reflectance due to interference in thin oil film
// Based on Airy formula for thin-film interference
//
// Parameters:
//   cos_theta_i: cosine of incident angle (in air)
//   oil_eta: refractive index of oil film
//   hair_eta: refractive index of hair substrate
//   thickness_nm: film thickness in nanometers
//
// Returns: RGB spectrum with interference colors
inline Spectrum thin_film_reflectance(
    Real cos_theta_i, Real oil_eta, Real hair_eta, Real thickness_nm) {

    // If no film thickness, return standard Fresnel
    if (thickness_nm <= Real(0)) {
        Real F = fresnel_dielectric(cos_theta_i, oil_eta);
        return make_const_spectrum(F);
    }

    // Snell's law: sin(theta_t) = sin(theta_i) / oil_eta
    Real sin_theta_i = sqrt(max(Real(0), 1 - cos_theta_i * cos_theta_i));
    Real sin_theta_t = sin_theta_i / oil_eta;

    // Total internal reflection check
    if (sin_theta_t >= Real(1)) {
        return make_const_spectrum(Real(1));
    }

    Real cos_theta_t = sqrt(max(Real(0), 1 - sin_theta_t * sin_theta_t));

    // Fresnel reflectance at air-oil interface (r1) and oil-hair interface (r2)
    // Using s-polarization (perpendicular) - simplified, could average s and p
    Real r1 = (cos_theta_i - oil_eta * cos_theta_t) /
              (cos_theta_i + oil_eta * cos_theta_t);

    // For oil-hair interface, compute angle in oil
    Real sin_theta_hair = sin_theta_t * oil_eta / hair_eta;
    Real cos_theta_hair = (sin_theta_hair < Real(1)) ?
                          sqrt(max(Real(0), 1 - sin_theta_hair * sin_theta_hair)) : Real(0);

    Real r2 = (oil_eta * cos_theta_t - hair_eta * cos_theta_hair) /
              (oil_eta * cos_theta_t + hair_eta * cos_theta_hair);

    // Optical path difference: 2 * n * d * cos(theta_t)
    Real opd = 2 * oil_eta * thickness_nm * cos_theta_t;

    // RGB wavelengths in nanometers
    const Real lambda_R = Real(650);  // Red
    const Real lambda_G = Real(550);  // Green
    const Real lambda_B = Real(450);  // Blue

    Spectrum result;
    Real wavelengths[3] = {lambda_R, lambda_G, lambda_B};

    for (int i = 0; i < 3; ++i) {
        // Phase difference: 2π * OPD / λ
        // Additional π phase shift at air-oil interface (low to high IOR)
        Real phase = c_TWOPI * opd / wavelengths[i] + c_PI;

        // Airy formula for thin-film reflectance:
        // R = (r1² + r2² + 2*r1*r2*cos(phase)) / (1 + r1²*r2² + 2*r1*r2*cos(phase))
        Real r1_sq = r1 * r1;
        Real r2_sq = r2 * r2;
        Real cos_phase = cos(phase);

        Real numerator = r1_sq + r2_sq + 2 * r1 * r2 * cos_phase;
        Real denominator = 1 + r1_sq * r2_sq + 2 * r1 * r2 * cos_phase;

        result[i] = std::clamp(numerator / denominator, Real(0), Real(1));
    }

    return result;
}

// Evaluate HairBCSDF with effective eta = hair_eta / oil_eta
inline Spectrum eval_hair_with_eta(
    const Vector3 &wi, const Vector3 &wo,
    const Spectrum &sigma_a, Real beta_m, Real beta_n,
    Real eta_effective, Real alpha,
    Real scale_R, Real scale_TT, Real scale_TRT) {

    // Compute hair angles
    Real sin_theta_i, cos_theta_i, phi_i;
    Real sin_theta_o, cos_theta_o, phi_o;
    hair::compute_hair_angles(wi, sin_theta_i, cos_theta_i, phi_i);
    hair::compute_hair_angles(wo, sin_theta_o, cos_theta_o, phi_o);

    // Variance for longitudinal scattering
    Real v_R = beta_m * beta_m;
    Real v_TT = v_R / 2;
    Real v_TRT = 2 * v_R;

    // Azimuthal scale
    Real s = max(beta_n * Real(0.626657), Real(0.1));

    // Phi difference
    Real phi = hair::compute_phi(phi_i, phi_o);

    // Use h=0 approximation (ray hits center of fiber)
    Real h = Real(0);
    Real gamma_o, gamma_t;
    hair::compute_gammas(h, eta_effective, gamma_o, gamma_t);

    // Absorption through fiber
    Spectrum T = hair::absorption_through_fiber(sigma_a, cos_theta_o, gamma_t, 1);

    // Cuticle tilt
    Real alpha_rad = alpha * c_PI / 180;

    Spectrum fsum = make_zero_spectrum();

    // R lobe (p=0) - uses eta_effective for Fresnel
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) - 2 * alpha_rad;
        Real sin_theta_i_R = sin(theta_i_shifted);
        Real cos_theta_i_R = cos(theta_i_shifted);

        Real M_R = hair::M_longitudinal(sin_theta_i_R, cos_theta_i_R,
                                        sin_theta_o, cos_theta_o, v_R);
        Real N_R = hair::trimmed_logistic(phi, s, -c_PI, c_PI);
        Spectrum A_R = hair::Ap(0, cos_theta_o, eta_effective, h, T);

        fsum += scale_R * A_R * M_R * N_R;
    }

    // TT lobe (p=1)
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + alpha_rad;
        Real sin_theta_i_TT = sin(theta_i_shifted);
        Real cos_theta_i_TT = cos(theta_i_shifted);

        Real M_TT = hair::M_longitudinal(sin_theta_i_TT, cos_theta_i_TT,
                                         sin_theta_o, cos_theta_o, v_TT);
        Real phi_TT = hair::compute_phi_p(1, gamma_o, gamma_t);
        Real dphi_TT = hair::compute_phi(phi, phi_TT);
        Real N_TT = hair::trimmed_logistic(dphi_TT, s, -c_PI, c_PI);
        Spectrum A_TT = hair::Ap(1, cos_theta_o, eta_effective, h, T);

        fsum += scale_TT * A_TT * M_TT * N_TT;
    }

    // TRT lobe (p=2)
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + 4 * alpha_rad;
        Real sin_theta_i_TRT = sin(theta_i_shifted);
        Real cos_theta_i_TRT = cos(theta_i_shifted);

        Real M_TRT = hair::M_longitudinal(sin_theta_i_TRT, cos_theta_i_TRT,
                                          sin_theta_o, cos_theta_o, v_TRT);
        Real phi_TRT = hair::compute_phi_p(2, gamma_o, gamma_t);
        Real dphi_TRT = hair::compute_phi(phi, phi_TRT);
        Real N_TRT = hair::trimmed_logistic(dphi_TRT, s, -c_PI, c_PI);
        Spectrum A_TRT = hair::Ap(2, cos_theta_o, eta_effective, h, T);

        fsum += scale_TRT * A_TRT * M_TRT * N_TRT;
    }

    return fsum;
}

// PDF for hair substrate sampling with modified eta
inline Real pdf_hair_with_eta(
    const Vector3 &wi, const Vector3 &wo,
    Real beta_m, Real beta_n,
    Real eta_effective, Real alpha,
    Real scale_R, Real scale_TT, Real scale_TRT) {

    Real sin_theta_i, cos_theta_i, phi_i;
    Real sin_theta_o, cos_theta_o, phi_o;
    hair::compute_hair_angles(wi, sin_theta_i, cos_theta_i, phi_i);
    hair::compute_hair_angles(wo, sin_theta_o, cos_theta_o, phi_o);

    Real v_R = beta_m * beta_m;
    Real v_TT = v_R / 2;
    Real v_TRT = 2 * v_R;

    Real s = max(beta_n * Real(0.626657), Real(0.1));
    Real phi = hair::compute_phi(phi_i, phi_o);

    Real h = Real(0);
    Real gamma_o, gamma_t;
    hair::compute_gammas(h, eta_effective, gamma_o, gamma_t);

    Real alpha_rad = alpha * c_PI / 180;

    // Lobe weights
    Real total_weight = scale_R + scale_TT + scale_TRT;
    if (total_weight < Real(1e-6)) total_weight = Real(1);

    Real pdf = 0;

    // R lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) - 2 * alpha_rad;
        Real M_R = hair::M_longitudinal(sin(theta_i_shifted), cos(theta_i_shifted),
                                        sin_theta_o, cos_theta_o, v_R);
        Real N_R = hair::trimmed_logistic(phi, s, -c_PI, c_PI);
        pdf += (scale_R / total_weight) * M_R * N_R;
    }

    // TT lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + alpha_rad;
        Real M_TT = hair::M_longitudinal(sin(theta_i_shifted), cos(theta_i_shifted),
                                         sin_theta_o, cos_theta_o, v_TT);
        Real phi_TT = hair::compute_phi_p(1, gamma_o, gamma_t);
        Real dphi_TT = hair::compute_phi(phi, phi_TT);
        Real N_TT = hair::trimmed_logistic(dphi_TT, s, -c_PI, c_PI);
        pdf += (scale_TT / total_weight) * M_TT * N_TT;
    }

    // TRT lobe PDF
    {
        Real theta_i_shifted = asin(std::clamp(sin_theta_i, Real(-1), Real(1))) + 4 * alpha_rad;
        Real M_TRT = hair::M_longitudinal(sin(theta_i_shifted), cos(theta_i_shifted),
                                          sin_theta_o, cos_theta_o, v_TRT);
        Real phi_TRT = hair::compute_phi_p(2, gamma_o, gamma_t);
        Real dphi_TRT = hair::compute_phi(phi, phi_TRT);
        Real N_TRT = hair::trimmed_logistic(dphi_TRT, s, -c_PI, c_PI);
        pdf += (scale_TRT / total_weight) * M_TRT * N_TRT;
    }

    return pdf;
}

} // namespace oil_hair

// ============================================================================
// MACRO-NORMAL RESOLUTION
// ============================================================================
// CRITICAL: For curve primitives, geometric_normal is the CYLINDER normal,
// NOT the underlying skin mesh normal. The Hair BSDF has no knowledge of
// the skin's existence.
//
// Solution: Use BAKED skin normals from Blender export (stored per-strand).
// Fallback to procedural estimation for legacy data without baked normals.
// ============================================================================

// Fallback: Procedural ellipsoid estimation (legacy, less accurate)
inline Vector3 estimate_macro_normal_procedural(const Vector3 &p) {
    // Seal body approximation: elongated ellipsoid along Z axis
    Vector3 center = Vector3{Real(0), Real(0.08), Real(0)};
    Vector3 offset = p - center;

    Real scale_x = Real(1.0);
    Real scale_y = Real(1.2);
    Real scale_z = Real(0.5);

    Vector3 scaled = Vector3{
        offset.x * scale_x,
        offset.y * scale_y,
        offset.z * scale_z
    };

    Real len = length(scaled);
    if (len < Real(1e-6)) {
        return Vector3{Real(0), Real(1), Real(0)};
    }
    return normalize(scaled);
}

// Get macro-normal: prefer baked skin normal, fallback to procedural
inline Vector3 get_macro_normal(const PathVertex &vertex) {
    if (vertex.has_skin_normal) {
        // Use the TRUE skin normal baked from Blender
        // This is the exact normal of the underlying mesh at the hair root
        return normalize(vertex.skin_normal);
    }
    // Fallback for legacy data without baked normals
    return estimate_macro_normal_procedural(vertex.position);
}

// ============================================================================
// Evaluate Oil-Coated Hair BCSDF (v30: Artistic Oil Sheen)
// ============================================================================
// CRITICAL: Creates the "wet seal" illusion with:
// 1. GGX specular on BAKED SKIN NORMAL with ARTISTIC Fresnel bias
// 2. Darkened substrate (IOR matching reduces internal scattering)
//
// v30 FIX:
// - Add artistic F0 boost (min 8% reflection) for visible oil sheen
// - Roughness bounds [0.08, 0.4] - glossy wet look
// - Physical Fresnel + artistic bias for that "slick" appearance
// ============================================================================
Spectrum eval_op::operator()(const OilCoatedHairBCSDF &bsdf) const {
    // =========================================================================
    // BUILD MACRO-FRAME FROM BAKED SKIN NORMAL
    // =========================================================================
    Vector3 macro_n = get_macro_normal(vertex);
    if (dot(macro_n, dir_in) < 0) {
        macro_n = -macro_n;
    }
    Frame macro_frame(macro_n);  // Frame.n = skin_normal

    // Transform ALL directions to this macro-frame
    Vector3 wi = to_local(macro_frame, dir_in);
    Vector3 wo = to_local(macro_frame, dir_out);

    Real n_dot_in = wi.z;
    Real n_dot_out = wo.z;

    // Backface culling
    if (n_dot_in <= 0 || n_dot_out <= 0) {
        return make_zero_spectrum();
    }

    // =========================================================================
    // v32 FIX: Use surface_uv for texture lookup (the ROOT UV from mesh)
    // This is CRITICAL - vertex.uv gets overwritten by curve parameterization
    // surface_uv contains the actual mesh texture coordinate for spotted pattern
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
    if (DEBUG_OIL_UV_AS_COLOR) {
        // Return UV as color: R = u, G = v, B = 0.3 if has_surface_uv
        Real b = vertex.has_surface_uv ? Real(0.3) : Real(0);
        return Spectrum{tex_uv.x, tex_uv.y, b};
    }

    // Get material parameters - use tex_uv for spotted fur pattern
    Spectrum base_color = eval(bsdf.sigma_a, tex_uv, vertex.uv_screen_size, texture_pool);
    Real oil_roughness = eval(bsdf.oil_roughness, tex_uv, vertex.uv_screen_size, texture_pool);

    // v45: Debug output to verify texture sampling
    static bool first_oil_eval = true;
    if (first_oil_eval) {
        first_oil_eval = false;
        std::cerr << "[OilHair BSDF Debug] tex_uv=(" << tex_uv.x << "," << tex_uv.y << ")"
                  << ", use_reflectance=" << bsdf.use_reflectance
                  << ", base_color_raw=(" << base_color[0] << "," << base_color[1] << "," << base_color[2] << ")"
                  << std::endl;
    }

    // Clamp base color
    if (bsdf.use_reflectance) {
        for (int i = 0; i < 3; ++i) {
            base_color[i] = std::clamp(base_color[i], Real(0.01), Real(0.99));
        }
    }

    // =========================================================================
    // v32: PATH SPACE FILTERING + GLOSSY ROUGHNESS FOR WET LOOK
    // =========================================================================
    // Base roughness bounds: 0.08 = very slick, 0.4 = damp
    // Path space filtering: increase minimum roughness on indirect bounces
    // to prevent PDF singularities (1/cos²θ) that cause fireflies
    // =========================================================================
    Real min_roughness = Real(0.08);  // Base minimum for camera rays
    if (vertex.bounce_depth > 0) {
        // CRITICAL: Higher minimum roughness on indirect bounces
        // This prevents fireflies from near-grazing angle reflections
        min_roughness = Real(0.25);
    }
    oil_roughness = std::clamp(oil_roughness, min_roughness, Real(0.4));

    Spectrum result = make_zero_spectrum();

    // =========================================================================
    // COMPONENT 1: OIL FILM SPECULAR (Artistic Fresnel Bias)
    // =========================================================================
    Vector3 h = normalize(wi + wo);
    Real n_dot_h = max(h.z, Real(0.001));
    Real h_dot_out = max(dot(h, wo), Real(0.001));

    // =========================================================================
    // v30 FIX: ARTISTIC FRESNEL WITH MINIMUM REFLECTION
    // =========================================================================
    // Physical Fresnel is too weak at normal incidence for visible "oil sheen"
    // Add an artistic bias to ensure minimum 8% reflection for that slick look
    Real eta = bsdf.oil_eta;
    Real F0_physical = ((eta - Real(1)) / (eta + Real(1)));
    F0_physical = F0_physical * F0_physical;

    // Artistic boost: ensure at least 8% reflection at normal incidence
    // This creates the visible "oily sheen" that makes wet fur look wet
    Real F0_min = Real(0.08);  // Minimum 8% reflection for oil sheen visibility
    Real F0 = max(F0_physical, F0_min);

    // Schlick approximation for Fresnel
    Real F_oil = F0 + (Real(1) - F0) * pow(Real(1) - h_dot_out, 5);

    // GGX microfacet BRDF
    Real denom = 4 * max(n_dot_in, Real(0.01)) * max(n_dot_out, Real(0.01));
    Real D = GTR2(n_dot_h, oil_roughness);
    Real G = smith_masking_gtr2(wi, oil_roughness) *
             smith_masking_gtr2(wo, oil_roughness);

    Real microfacet = (D * G) / denom;

    // =========================================================================
    // DEBUG TASK 2: Isolate oil specular vs substrate
    // =========================================================================
    if (!DEBUG_DISABLE_OIL_SPECULAR) {
        // v44 FIX: Use oil_specular_scale to control specular intensity
        // This parameter was being parsed but never used!
        // Scale > 1.0 boosts reflections, < 1.0 reduces them
        // Normalize by 18.0 (the XML default) to maintain backward compatibility
        Real specular_intensity = bsdf.oil_specular_scale / Real(18.0);
        Real oil_contrib = specular_intensity * F_oil * microfacet;

        // v44: Tint the specular slightly with base_color for spot preservation
        // This ensures spots show through even in specular highlights
        Real spot_tint = Real(0.15);  // 15% texture color in highlights
        Spectrum specular_color = make_const_spectrum(Real(1) - spot_tint) + base_color * spot_tint;

        result += specular_color * oil_contrib * n_dot_out;
    }

    // If DEBUG_OIL_SPECULAR_ONLY, skip substrate entirely
    if (DEBUG_OIL_SPECULAR_ONLY) {
        return result;
    }

    // =========================================================================
    // COMPONENT 2: TEXTURED SUBSTRATE (v44: INCREASED VISIBILITY)
    // =========================================================================
    // v44 FIX: wet_darkening was 0.10 (10%) which made spots invisible!
    // Increased to 0.40 (40%) to preserve spotted fur pattern visibility.
    // The spots are a critical visual feature - they must be visible.
    // =========================================================================
    Real transmission = (Real(1) - F_oil) * (Real(1) - F_oil);

    // v44: wet_darkening increased from 0.10 to 0.40 for spot visibility
    Real wet_darkening = Real(0.40);  // 40% of dry albedo - SPOTS MUST BE VISIBLE!

    // Lambertian on macro-surface with TEXTURED base_color
    Spectrum substrate = wet_darkening * base_color * (n_dot_out / c_PI);
    result += transmission * substrate;

    // =========================================================================
    // v36 NaN/Inf GUARD: Kill mathematical singularities at the source
    // =========================================================================
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(result[i]) || std::isinf(result[i])) {
            return make_zero_spectrum();  // Kill the entire contribution
        }
        result[i] = std::clamp(result[i], Real(0), Real(5));  // Hard clamp
    }

    return result;
}

// ============================================================================
// PDF for Oil-Coated Hair BCSDF (v30)
// ============================================================================
// Uses BAKED macro-normal from Blender export to match eval
// CRITICAL: Roughness bounds must match eval exactly for correct MIS
// ============================================================================
Real pdf_sample_bsdf_op::operator()(const OilCoatedHairBCSDF &bsdf) const {
    // Baked macro-normal from skin (same as eval)
    Vector3 macro_n = get_macro_normal(vertex);
    if (dot(macro_n, dir_in) < 0) {
        macro_n = -macro_n;
    }
    Frame macro_frame(macro_n);

    Vector3 wi = to_local(macro_frame, dir_in);
    Vector3 wo = to_local(macro_frame, dir_out);

    Real n_dot_in = wi.z;
    Real n_dot_out = wo.z;
    if (n_dot_in <= 0 || n_dot_out <= 0) {
        return Real(0);
    }

    // v32 FIX: Use surface_uv for texture lookup (matches eval)
    Vector2 tex_uv = vertex.has_surface_uv ? vertex.surface_uv : vertex.uv;
    // v45: Apply V-flip to match eval
    static const bool FLIP_V_COORDINATE = true;
    if (FLIP_V_COORDINATE && vertex.has_surface_uv) {
        tex_uv.y = Real(1) - tex_uv.y;
    }

    Real oil_roughness = eval(bsdf.oil_roughness, tex_uv, vertex.uv_screen_size, texture_pool);

    // v32: PATH SPACE FILTERING - must match eval exactly for correct MIS!
    Real min_roughness = Real(0.08);
    if (vertex.bounce_depth > 0) {
        min_roughness = Real(0.25);
    }
    oil_roughness = std::clamp(oil_roughness, min_roughness, Real(0.4));

    // v30: Higher specular weight for glossy wet appearance
    Real weight_oil = Real(0.6);     // 60% specular sampling
    Real weight_diffuse = Real(0.4); // 40% diffuse sampling
    Real total_weight = weight_oil + weight_diffuse;
    weight_oil /= total_weight;
    weight_diffuse /= total_weight;

    Real pdf = 0;

    // Oil GGX PDF on macro-surface
    Vector3 h = normalize(wi + wo);
    Real n_dot_h = max(h.z, Real(0.001));
    if (n_dot_h > 0) {
        Real D = GTR2(n_dot_h, oil_roughness);
        Real G_in = smith_masking_gtr2(wi, oil_roughness);
        Real pdf_oil = (D * G_in) / (4 * max(n_dot_in, Real(0.01)));
        pdf += weight_oil * pdf_oil;
    }

    // Diffuse PDF (cosine-weighted on macro-surface)
    pdf += weight_diffuse * (n_dot_out / c_PI);

    return max(pdf, Real(1e-6));
}

// ============================================================================
// Sample Oil-Coated Hair BCSDF (v30)
// ============================================================================
// Samples on BAKED macro-normal from Blender export to match eval
// CRITICAL: Roughness bounds and weights must match pdf/eval for correct MIS
// ============================================================================
std::optional<BSDFSampleRecord> sample_bsdf_op::operator()(const OilCoatedHairBCSDF &bsdf) const {
    // Baked macro-normal from skin (same as eval)
    Vector3 macro_n = get_macro_normal(vertex);
    if (dot(macro_n, dir_in) < 0) {
        macro_n = -macro_n;
    }
    Frame macro_frame(macro_n);

    Vector3 wi = to_local(macro_frame, dir_in);
    Real n_dot_in = wi.z;
    if (n_dot_in <= 0) {
        return {};
    }

    // v32 FIX: Use surface_uv for texture lookup (matches eval/pdf)
    Vector2 tex_uv = vertex.has_surface_uv ? vertex.surface_uv : vertex.uv;
    // v45: Apply V-flip to match eval
    static const bool FLIP_V_COORDINATE = true;
    if (FLIP_V_COORDINATE && vertex.has_surface_uv) {
        tex_uv.y = Real(1) - tex_uv.y;
    }

    Real oil_roughness = eval(bsdf.oil_roughness, tex_uv, vertex.uv_screen_size, texture_pool);

    // v32: PATH SPACE FILTERING - must match eval/pdf exactly for correct MIS!
    Real min_roughness = Real(0.08);
    if (vertex.bounce_depth > 0) {
        min_roughness = Real(0.25);
    }
    oil_roughness = std::clamp(oil_roughness, min_roughness, Real(0.4));

    // v30: Higher specular weight - must match pdf weights exactly!
    Real weight_oil = Real(0.6);     // 60% specular sampling
    Real weight_diffuse = Real(0.4); // 40% diffuse sampling
    Real total_weight = weight_oil + weight_diffuse;
    weight_oil /= total_weight;
    weight_diffuse /= total_weight;

    Vector3 wo_local;
    Real roughness_out;

    if (rnd_param_w < weight_oil) {
        // Sample oil reflection from GGX lobe on macro-surface
        Real alpha = oil_roughness * oil_roughness;
        Vector3 local_h = sample_visible_normals(wi, alpha, rnd_param_uv);

        wo_local = -wi + 2 * dot(wi, local_h) * local_h;

        if (wo_local.z <= 0) {
            return {};
        }

        roughness_out = oil_roughness;
    } else {
        // Sample diffuse (cosine-weighted on macro-surface)
        wo_local = sample_cos_hemisphere(rnd_param_uv);

        if (wo_local.z <= 0) {
            return {};
        }

        roughness_out = Real(1);
    }

    Vector3 wo = to_world(macro_frame, wo_local);

    return BSDFSampleRecord{wo, Real(0), roughness_out};
}

// ============================================================================
// Get texture for debugging
// ============================================================================
TextureSpectrum get_texture_op::operator()(const OilCoatedHairBCSDF &bsdf) const {
    return bsdf.sigma_a;
}
