#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyBSDF &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Get all texture parameters
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specular_transmission = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real subsurface = eval(bsdf.subsurface, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specular = eval(bsdf.specular, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real specular_tint = eval(bsdf.specular_tint, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen = eval(bsdf.sheen, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen_tint = eval(bsdf.sheen_tint, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat_gloss = eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Check if we're inside the object (Equation 21)
    bool inside = dot(vertex.geometric_normal, dir_in) < 0;

    Spectrum result = make_zero_spectrum();

    // If inside object, only evaluate glass lobe
    if (!inside) {
        // Evaluate diffuse lobe (Equation 19)
        if (metallic < 1.0 && specular_transmission < 1.0) {
            DisneyDiffuse diffuse{bsdf.base_color, bsdf.roughness, bsdf.subsurface};
            Spectrum f_diffuse = eval_op{dir_in, dir_out, vertex, texture_pool, dir}(diffuse);
            result += (1.0 - specular_transmission) * (1.0 - metallic) * f_diffuse;
        }

        // Evaluate sheen lobe (Equation 19)
        if (metallic < 1.0 && sheen > 0) {
            DisneySheen sheen_bsdf{bsdf.base_color, bsdf.sheen_tint};
            Spectrum f_sheen = eval_op{dir_in, dir_out, vertex, texture_pool, dir}(sheen_bsdf);
            result += (1.0 - metallic) * sheen * f_sheen;
        }

        // Evaluate modified metal lobe with dielectric specular (Equation 19 + 20)
        {
            // Compute Ctint for specular tinting
            Real lum = luminance(base_color);
            Spectrum Ctint = lum > 0 ? base_color / lum : make_const_spectrum(1.0);

            // Compute Ks (blend between white and tinted specular)
            Spectrum Ks = (1.0 - specular_tint) + specular_tint * Ctint;

            // Compute R0 for dielectric (eta = 1.5)
            Real R0 = (1.5 - 1.0) * (1.5 - 1.0) / ((1.5 + 1.0) * (1.5 + 1.0));

            // Modified base color for metal with dielectric specular (Equation 20)
            Spectrum C0 = specular * R0 * (1.0 - metallic) * Ks + metallic * base_color;

            // Create modified metal material with C0 as base color
            DisneyMetal metal{make_constant_spectrum_texture(C0), bsdf.roughness, bsdf.anisotropic};
            Spectrum f_metal = eval_op{dir_in, dir_out, vertex, texture_pool, dir}(metal);
            result += (1.0 - specular_transmission * (1.0 - metallic)) * f_metal;
        }

        // Evaluate clearcoat lobe (Equation 19)
        if (clearcoat > 0) {
            DisneyClearcoat clearcoat_bsdf{bsdf.clearcoat_gloss};
            Spectrum f_clearcoat = eval_op{dir_in, dir_out, vertex, texture_pool, dir}(clearcoat_bsdf);
            result += 0.25 * clearcoat * f_clearcoat;
        }
    }

    // Evaluate glass lobe (Equation 19)
    if (metallic < 1.0 && specular_transmission > 0) {
        DisneyGlass glass{bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        Spectrum f_glass = eval_op{dir_in, dir_out, vertex, texture_pool, dir}(glass);
        result += (1.0 - metallic) * specular_transmission * f_glass;
    }

    return result;
}

Real pdf_sample_bsdf_op::operator()(const DisneyBSDF &bsdf) const {
    bool reflect = dot(vertex.geometric_normal, dir_in) *
                   dot(vertex.geometric_normal, dir_out) > 0;
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Get texture parameters
    Real specular_transmission = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen = eval(bsdf.sheen, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Check if we're inside the object
    bool inside = dot(vertex.geometric_normal, dir_in) < 0;

    // Compute sampling weights (Equation 22)
    Real diffuse_weight = 0;
    Real metal_weight = 0;
    Real glass_weight = 0;
    Real clearcoat_weight = 0;
    Real sheen_weight = 0;

    if (inside) {
        // Inside object: only glass lobe
        glass_weight = 1.0;
    } else {
        // Outside object: compute weights for all lobes
        diffuse_weight = (1.0 - metallic) * (1.0 - specular_transmission);
        metal_weight = 1.0 - specular_transmission * (1.0 - metallic);
        glass_weight = (1.0 - metallic) * specular_transmission;
        clearcoat_weight = 0.25 * clearcoat;
        sheen_weight = (1.0 - metallic) * sheen;  // Sheen uses same distribution as diffuse
    }

    // Normalize weights (include sheen since it contributes to eval)
    Real total_weight = diffuse_weight + metal_weight + glass_weight + clearcoat_weight + sheen_weight;
    if (total_weight <= 0) {
        return 0;
    }

    diffuse_weight /= total_weight;
    metal_weight /= total_weight;
    glass_weight /= total_weight;
    clearcoat_weight /= total_weight;
    sheen_weight /= total_weight;

    // Compute weighted PDF
    Real pdf = 0;

    if (diffuse_weight > 0) {
        DisneyDiffuse diffuse{bsdf.base_color, bsdf.roughness, bsdf.subsurface};
        pdf += diffuse_weight * pdf_sample_bsdf_op{dir_in, dir_out, vertex, texture_pool, dir}(diffuse);
    }

    // Sheen uses same distribution as diffuse (cosine-weighted)
    if (sheen_weight > 0 && reflect) {
        DisneySheen sheen_bsdf{bsdf.base_color, bsdf.sheen_tint};
        // Sheen is evaluated on reflection only, use cosine PDF
        Real cos_theta = fmax(dot(frame.n, dir_out), Real(0));
        pdf += sheen_weight * cos_theta / c_PI;
    }

    if (metal_weight > 0) {
        // Use original base_color for metal PDF (not modified C0)
        DisneyMetal metal{bsdf.base_color, bsdf.roughness, bsdf.anisotropic};
        pdf += metal_weight * pdf_sample_bsdf_op{dir_in, dir_out, vertex, texture_pool, dir}(metal);
    }

    if (glass_weight > 0) {
        DisneyGlass glass{bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        pdf += glass_weight * pdf_sample_bsdf_op{dir_in, dir_out, vertex, texture_pool, dir}(glass);
    }

    if (clearcoat_weight > 0) {
        DisneyClearcoat clearcoat_bsdf{bsdf.clearcoat_gloss};
        pdf += clearcoat_weight * pdf_sample_bsdf_op{dir_in, dir_out, vertex, texture_pool, dir}(clearcoat_bsdf);
    }

    return pdf;
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyBSDF &bsdf) const {
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) * dot(vertex.geometric_normal, dir_in) < 0) {
        frame = -frame;
    }
    // Get texture parameters
    Real specular_transmission = eval(bsdf.specular_transmission, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real metallic = eval(bsdf.metallic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real clearcoat = eval(bsdf.clearcoat, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real sheen = eval(bsdf.sheen, vertex.uv, vertex.uv_screen_size, texture_pool);

    // Check if we're inside the object
    bool inside = dot(vertex.geometric_normal, dir_in) < 0;

    // Compute sampling weights (Equation 22)
    Real diffuse_weight = 0;
    Real metal_weight = 0;
    Real glass_weight = 0;
    Real clearcoat_weight = 0;
    Real sheen_weight = 0;

    if (inside) {
        // Inside object: only glass lobe
        glass_weight = 1.0;
    } else {
        // Outside object: compute weights for all lobes
        diffuse_weight = (1.0 - metallic) * (1.0 - specular_transmission);
        metal_weight = 1.0 - specular_transmission * (1.0 - metallic);
        glass_weight = (1.0 - metallic) * specular_transmission;
        clearcoat_weight = 0.25 * clearcoat;
        sheen_weight = (1.0 - metallic) * sheen;  // Sheen sampled with diffuse
    }

    // Normalize weights (combine diffuse and sheen for sampling since same distribution)
    Real diffuse_sheen_weight = diffuse_weight + sheen_weight;
    Real total_weight = diffuse_sheen_weight + metal_weight + glass_weight + clearcoat_weight;
    if (total_weight <= 0) {
        return {};
    }

    diffuse_sheen_weight /= total_weight;
    metal_weight /= total_weight;
    glass_weight /= total_weight;
    clearcoat_weight /= total_weight;

    // Randomly select a lobe based on weights and rescale random number
    Real u = rnd_param_w;
    Real cumulative = 0;
    Real prev_cumulative = 0;

    // Sample diffuse lobe (includes sheen since same distribution)
    prev_cumulative = cumulative;
    if (u < (cumulative += diffuse_sheen_weight)) {
        // Rescale rnd_param_w to [0,1] for the selected lobe
        Real rescaled_w = (u - prev_cumulative) / diffuse_sheen_weight;
        DisneyDiffuse diffuse{bsdf.base_color, bsdf.roughness, bsdf.subsurface};
        return sample_bsdf_op{dir_in, vertex, texture_pool, rnd_param_uv, rescaled_w, dir}(diffuse);
    }

    // Sample metal lobe
    prev_cumulative = cumulative;
    if (u < (cumulative += metal_weight)) {
        Real rescaled_w = (u - prev_cumulative) / metal_weight;
        DisneyMetal metal{bsdf.base_color, bsdf.roughness, bsdf.anisotropic};
        return sample_bsdf_op{dir_in, vertex, texture_pool, rnd_param_uv, rescaled_w, dir}(metal);
    }

    // Sample glass lobe
    prev_cumulative = cumulative;
    if (u < (cumulative += glass_weight)) {
        Real rescaled_w = (u - prev_cumulative) / glass_weight;
        DisneyGlass glass{bsdf.base_color, bsdf.roughness, bsdf.anisotropic, bsdf.eta};
        return sample_bsdf_op{dir_in, vertex, texture_pool, rnd_param_uv, rescaled_w, dir}(glass);
    }

    // Sample clearcoat lobe
    {
        Real rescaled_w = (u - cumulative) / clearcoat_weight;
        DisneyClearcoat clearcoat_bsdf{bsdf.clearcoat_gloss};
        return sample_bsdf_op{dir_in, vertex, texture_pool, rnd_param_uv, rescaled_w, dir}(clearcoat_bsdf);
    }
}

TextureSpectrum get_texture_op::operator()(const DisneyBSDF &bsdf) const {
    return bsdf.base_color;
}
