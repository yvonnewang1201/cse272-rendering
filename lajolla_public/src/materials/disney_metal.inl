#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyMetal &bsdf) const {
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
    // Homework 1: implement this!
    // The color and settings from the material
    Spectrum base_color = eval(bsdf.base_color, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    
    Vector3 h = normalize(dir_in+dir_out);
    Vector3 h_local=to_local(frame, h);

    Real n_dot_in=fabs(dot(frame.n, dir_in));
    Real n_dot_out=fabs(dot(frame.n, dir_out));
    Real h_dot_out=fabs(dot(h, dir_out));

    //Fm = baseColor + (1 −baseColor)(1 −|h ·ωout|)^5
    Spectrum F_m=base_color+(1.0-base_color)*pow(1-h_dot_out, 5);

    //aspect = √1 −0.9anisotropic
    Real aspect=sqrt(1.0-0.9*anisotropic);
    //αx = max(αmin,roughness^2/aspect)
    Real alpha_x=max(0.0001, roughness*roughness/aspect);
    //αy = max(αmin,roughness^2 ·aspect)
    Real alpha_y=max(0.0001, roughness*roughness*aspect);
    //Dm=1/(παxαy(h_l_x^2/αx^2+ h_l_y^2/αy^2+ h_l_z^2)^2)
    Real term=(h_local.x*h_local.x)/(alpha_x*alpha_x)+(h_local.y*h_local.y)/(alpha_y*alpha_y)+h_local.z*h_local.z;
    Real D_m=1.0/(c_PI*alpha_x*alpha_y*(term*term));
    
    Vector3 win_local=to_local(frame, dir_in);
    Vector3 wout_local=to_local(frame, dir_out);
    //Λ(ω)=(√((1 + ((ωl.x·αx)^2+(ωl.y·αy)^2))/ωl.z^2) −1)/2
    Real lambda_win=(sqrt(1.0+((win_local.x*alpha_x)*(win_local.x*alpha_x)+(win_local.y*alpha_y)*(win_local.y*alpha_y))/(win_local.z*win_local.z))-1)/2.0;
    Real lambda_wout=(sqrt(1.0+((wout_local.x*alpha_x)*(wout_local.x*alpha_x)+(wout_local.y*alpha_y)*(wout_local.y*alpha_y))/(wout_local.z*wout_local.z))-1)/2.0;
    //G(ω) = 1/1 + Λ(ω)
    Real G_win=1.0/(1.0+lambda_win);
    Real G_wout=1.0/(1.0+lambda_wout);
    //Gm = G(ωin)G(ωout)
    Real G_m=G_win*G_wout;
    //fmetal = FmDmGm/4|n ·ωin| 
    Spectrum f_metal=(F_m*D_m*G_m*n_dot_out)/(4*n_dot_in);
    return f_metal;
}

Real pdf_sample_bsdf_op::operator()(const DisneyMetal &bsdf) const {
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
    // Homework 1: implement this!
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);
    Vector3 h = normalize(dir_in+dir_out);
    Vector3 h_local=to_local(frame, h);

    Real n_dot_in=fabs(dot(frame.n, dir_in));
    Real h_dot_out=fabs(dot(h, dir_out));

    //aspect = √1 −0.9anisotropic
    Real aspect=sqrt(1.0-0.9*anisotropic);
    //αx = max(αmin,roughness^2/aspect)
    Real alpha_x=max(0.0001, roughness*roughness/aspect);
    //αy = max(αmin,roughness^2 ·aspect)
    Real alpha_y=max(0.0001, roughness*roughness*aspect);
    //Dm=1/(παxαy(h_l_x^2/αx^2+ h_l_y^2/αy^2+ h_l_z^2)^2)
    Real term=(h_local.x*h_local.x)/(alpha_x*alpha_x)+(h_local.y*h_local.y)/(alpha_y*alpha_y)+h_local.z*h_local.z;
    Real D_m=1.0/(c_PI*alpha_x*alpha_y*(term*term));
    
    Vector3 win_local=to_local(frame, dir_in);
    //Λ(ω)=(√((1 + ((ωl.x·αx)^2+(ωl.y·αy)^2))/ωl.z^2) −1)/2
    Real lambda_win=(sqrt(1.0+((win_local.x*alpha_x)*(win_local.x*alpha_x)+(win_local.y*alpha_y)*(win_local.y*alpha_y))/(win_local.z*win_local.z))-1)/2.0;
    //G(ω) = 1/1 + Λ(ω)
    Real G_win=1.0/(1.0+lambda_win);
    
    //pdf=D_m*G(win)/4|n ·ωin|
    return (D_m*G_win*h_dot_out)/(4.0*n_dot_in);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyMetal &bsdf) const {
    if (dot(vertex.geometric_normal, dir_in) < 0) {
        // No light below the surface
        return {};
    }
    // Flip the shading frame if it is inconsistent with the geometry normal
    Frame frame = vertex.shading_frame;
    if (dot(frame.n, dir_in) < 0) {
        frame = -frame;
    }
    // Homework 1: implement this!
    Real roughness = eval(bsdf.roughness, vertex.uv, vertex.uv_screen_size, texture_pool);
    Real anisotropic = eval(bsdf.anisotropic, vertex.uv, vertex.uv_screen_size, texture_pool);

    //aspect = √1 −0.9anisotropic
    Real aspect=sqrt(1.0-0.9*anisotropic);
    //αx = max(αmin,roughness^2/aspect)
    Real alpha_x=max(0.0001, roughness*roughness/aspect);
    //αy = max(αmin,roughness^2 ·aspect)
    Real alpha_y=max(0.0001, roughness*roughness*aspect);

    // Anisotropic visible normal sampling (Heitz 2018)
    Vector3 win_local=to_local(frame, dir_in);
    // Transform to isotropic configuration
    Vector3 win_stretched = normalize(Vector3{win_local.x * alpha_x, win_local.y * alpha_y, win_local.z});
    // Sample visible normal in isotropic space
    Vector3 local_micro_normal_iso = sample_visible_normals(win_stretched, Real(1.0), rnd_param_uv);
    // Transform back to anisotropic configuration
    Vector3 local_micro_normal = normalize(Vector3{
        local_micro_normal_iso.x * alpha_x,
        local_micro_normal_iso.y * alpha_y,
        local_micro_normal_iso.z
    });
    Vector3 wout_local = 2.0 * dot(local_micro_normal, win_local) * local_micro_normal - win_local;
    
    Vector3 dir_out = to_world(frame, wout_local);

    if (dot(vertex.geometric_normal, dir_out) <= 0) {
        return {};
    }

    return BSDFSampleRecord{
        dir_out,
        Real(0),  
        roughness
    };
}

TextureSpectrum get_texture_op::operator()(const DisneyMetal &bsdf) const {
    return bsdf.base_color;
}
