#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyGlass &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    // Homework 1: implement this!
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Clamp roughness to avoid numerical issues
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    // Determine if we're entering or exiting the material
    bool entering = dot(vertex.geometric_normal, dir_in) > 0;
    Real eta_ratio = entering ? bsdf.eta : (1.0 / bsdf.eta);

    Vector3 h;
    if (reflect) {
        h = normalize(dir_in + dir_out);
    } else {
        // For refraction: use the proper half-vector formula
        // h should be: -(eta_o * omega_o + eta_i * omega_i) / ||...||
        // When entering: eta_i=1 (air), eta_o=eta (glass)
        // So: h = -(eta * dir_out + dir_in) / ||...||
        h = normalize(dir_in + dir_out * eta_ratio);
    }

    // CRITICAL: Flip half-vector if it's below surface (for both reflection AND transmission)
    // This matches roughdielectric.inl behavior
    if (dot(h, frame.n) < 0) {
        h = -h;
    }

    Real n_dot_in = dot(frame.n, dir_in);
    Real n_dot_out = dot(frame.n, dir_out);
    Real n_dot_h = dot(frame.n, h);
    Real h_dot_in = dot(h, dir_in);
    Real h_dot_out = dot(h, dir_out);

    // Match roughdielectric - NO fabs for Fresnel
    Real F_g = fresnel_dielectric(h_dot_in, eta_ratio);

    // Anisotropic GGX
    Real aspect = sqrt(1.0 - 0.9 * anisotropic);
    Real alpha_x = max(Real(0.0001), roughness * roughness / aspect);
    Real alpha_y = max(Real(0.0001), roughness * roughness * aspect);
    Real alpha = sqrt(alpha_x * alpha_y);  // geometric mean for G term

    // Anisotropic D: Dm = 1/(π*αx*αy*(h_l_x^2/αx^2 + h_l_y^2/αy^2 + h_l_z^2)^2)
    Vector3 h_local = to_local(frame, h);
    Real term = (h_local.x * h_local.x) / (alpha_x * alpha_x) +
                (h_local.y * h_local.y) / (alpha_y * alpha_y) +
                h_local.z * h_local.z;
    Real D_g = 1.0 / (c_PI * alpha_x * alpha_y * (term * term));

    // Use geometric mean alpha for G (for numerical stability with refraction)
    Real G_g = smith_masking_gtr2(to_local(frame, dir_in), alpha) *
               smith_masking_gtr2(to_local(frame, dir_out), alpha);

    if (reflect) {
        // Reflection: Match roughdielectric - NO n_dot_out (disney_metal is wrong)
        return base_color * (F_g * D_g * G_g) / (4 * fabs(n_dot_in));
    } else {
        // Transmission: Disney glass uses sqrt(base_color)
        Spectrum transmission_color = sqrt(base_color);

        Real eta_factor = (dir == TransportDirection::TO_LIGHT) ? (1.0 / (eta_ratio * eta_ratio)) : 1.0;
        Real sqrt_denom = h_dot_in + eta_ratio * h_dot_out;

        if (fabs(sqrt_denom) < 1e-6) {
            return make_zero_spectrum();
        }

        // Include eta^2 term for energy balance
        Real numerator = (1.0 - F_g) * D_g * G_g * eta_ratio * eta_ratio * fabs(h_dot_out * h_dot_in);
        Real denominator = fabs(n_dot_in) * sqrt_denom * sqrt_denom;
        return transmission_color * eta_factor * numerator / denominator;
    }
}

Real pdf_sample_bsdf_op::operator()(const DisneyGlass &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    // Homework 1: implement this!
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Clamp roughness to avoid numerical issues
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    bool entering = dot(vertex.geometric_normal, dir_in) > 0;
    Real eta_ratio = entering ? bsdf.eta : (1.0 / bsdf.eta);

    Vector3 h;
    if (reflect) {
        h = normalize(dir_in + dir_out);
    } else {
        h = normalize(dir_in + dir_out * eta_ratio);
    }

    // Flip half-vector if it's below surface (match eval function logic)
    if (dot(h, frame.n) < 0) {
        h = -h;
    }

    Real h_dot_in = dot(h, dir_in);
    Real F = fresnel_dielectric(h_dot_in, eta_ratio);

    // Anisotropic GGX (same as eval)
    Real aspect = sqrt(1.0 - 0.9 * anisotropic);
    Real alpha_x = max(Real(0.0001), roughness * roughness / aspect);
    Real alpha_y = max(Real(0.0001), roughness * roughness * aspect);
    Real alpha = sqrt(alpha_x * alpha_y);  // geometric mean for G term

    // Anisotropic D
    Vector3 h_local = to_local(frame, h);
    Real term = (h_local.x * h_local.x) / (alpha_x * alpha_x) +
                (h_local.y * h_local.y) / (alpha_y * alpha_y) +
                h_local.z * h_local.z;
    Real D_g = 1.0 / (c_PI * alpha_x * alpha_y * (term * term));

    // Use geometric mean alpha for G (same as eval)
    Real G_in = smith_masking_gtr2(to_local(frame, dir_in), alpha);

    if (reflect) {
        // Match roughdielectric: NO h_dot_out in PDF!
        return (F * D_g * G_in) / (4 * fabs(dot(frame.n, dir_in)));
    } else {
        Real h_dot_out = dot(h, dir_out);
        Real sqrt_denom = h_dot_in + eta_ratio * h_dot_out;

        if (fabs(sqrt_denom) < 1e-6) {
            return 0;
        }

        Real dh_dout = eta_ratio * eta_ratio * h_dot_out / (sqrt_denom * sqrt_denom);
        return (1 - F) * D_g * G_in * fabs(dh_dout * h_dot_in / dot(frame.n, dir_in));
    }
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyGlass &bsdf) const {
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }

    // Homework 1: implement this!
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Clamp roughness to avoid numerical issues
    roughness = std::clamp(roughness, Real(0.01), Real(1));

    bool entering = dot(vertex.geometric_normal, dir_in) > 0;
    Real eta_ratio = entering ? bsdf.eta : (1.0 / bsdf.eta);

    // Anisotropic GGX (same as eval and pdf)
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real aspect = sqrt(1.0 - 0.9 * anisotropic);
    Real alpha_x = max(Real(0.0001), roughness * roughness / aspect);
    Real alpha_y = max(Real(0.0001), roughness * roughness * aspect);
    Real alpha = sqrt(alpha_x * alpha_y);  // geometric mean for sampling

    Vector3 local_dir_in = to_local(frame, dir_in);
    Vector3 local_micro_normal = sample_visible_normals(local_dir_in, alpha, rnd_param_uv);

    // Flip to ensure half-vector points into the same hemisphere as the surface normal
    // (consistent with eval/PDF logic)
    Vector3 half_vector = to_world(frame, local_micro_normal);
    if (dot(half_vector, frame.n) < 0) {
        local_micro_normal = -local_micro_normal;
        half_vector = -half_vector;
    }

    Real h_dot_in = dot(half_vector, dir_in);
    Real F = fresnel_dielectric(h_dot_in, eta_ratio);

    if (rnd_param_w <= F) {
        // Reflection
        Vector3 reflected = normalize(-dir_in + 2 * dot(dir_in, half_vector) * half_vector);
        // set eta to 0 since we are not transmitting
        return BSDFSampleRecord{reflected, Real(0) /* eta */, roughness};
    } else {
        // Refraction
        // https://en.wikipedia.org/wiki/Snell%27s_law#Vector_form
        // (note that our eta_ratio is eta2 / eta1, and l = -dir_in)
        Real h_dot_out_sq = 1 - (1 - h_dot_in * h_dot_in) / (eta_ratio * eta_ratio);
        if (h_dot_out_sq <= 0) {
            // Total internal reflection
            // This shouldn't really happen, as F will be 1 in this case.
            return {};
        }
        // flip half_vector if needed
        if (h_dot_in < 0) {
            half_vector = -half_vector;
        }
        Real h_dot_out = sqrt(h_dot_out_sq);
        Vector3 refracted = -dir_in / eta_ratio + (fabs(h_dot_in) / eta_ratio - h_dot_out) * half_vector;
        return BSDFSampleRecord{refracted, eta_ratio, roughness};
    }
}

TextureSpectrum get_texture_op::operator()(const DisneyGlass &bsdf) const {
    return bsdf.base_color;
}
