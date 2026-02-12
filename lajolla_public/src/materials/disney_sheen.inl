#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneySheen &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0 ||
            dot(vertex.geometric_normal, dir_out) < 0) {
        // No light below the surface
        return make_zero_spectrum();
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }

    // Get texture values
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen_tint = eval(bsdf.sheen_tint, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Compute half vector
    Vector3 h = normalize(dir_in + dir_out);

    // Compute Ctint (tinted color based on base color)
    Real lum = luminance(base_color);
    Spectrum Ctint = lum > 0 ? base_color / lum : make_const_spectrum(1.0);

    // Compute Csheen (blend between white and tinted)
    Spectrum Csheen = (1.0 - sheen_tint) + sheen_tint * Ctint;

    // Compute sheen BRDF: Csheen * (1 - |h·ωout|)^5 * |n·ωout|
    Real h_dot_out = fabs(dot(h, dir_out));
    Real n_dot_out = fabs(dot(frame.n, dir_out));
    Real fresnel_term = pow(1.0 - h_dot_out, 5.0);

    return Csheen * fresnel_term * n_dot_out;
}

Real pdf_sample_bsdf_op::operator()(const DisneySheen &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0 ||
            dot(vertex.geometric_normal, dir_out) < 0) {
        // No light below the surface
        return 0;
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }

    // Cosine hemisphere sampling PDF
    Real n_dot_out = dot(frame.n, dir_out);
    if (n_dot_out <= 0) {
        return 0;
    }
    return n_dot_out / c_PI;
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneySheen &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0) {
        // No light below the surface
        return {};
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }

    // Cosine hemisphere importance sampling (same as Lambertian)
    return BSDFSampleRecord{
        to_world(frame, sample_cos_hemisphere(rnd_param_uv)),
        Real(0) /* eta */, Real(1) /* roughness */};
}

TextureSpectrum get_texture_op::operator()(const DisneySheen &bsdf) const {
    return bsdf.base_color;
}
