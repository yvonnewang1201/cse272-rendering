#include "../microfacet.h"

Spectrum eval_op::operator()(const DisneyClearcoat &bsdf) const {
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
    Real clearcoatGloss= eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    
    Vector3 h=normalize(dir_in+dir_out);
    Vector3 h_local=to_local(frame, h);

    Real n_dot_in=fabs(dot(frame.n, dir_in));
    Real n_dot_out=fabs(dot(frame.n, dir_out));
    Real h_dot_out=fabs(dot(h, dir_out));

    //R0(η) = (η −1)^2/(η + 1)^2
    //Fc = R0(η = 1.5) + (1 −R0(η = 1.5)) (1 −|h ·ωout|)^5
    Real F_c=((0.5*0.5)/(2.5*2.5))+(1-((0.5*0.5)/(2.5*2.5)))*pow(1-h_dot_out, 5);
    
    //αg = (1 −clearcoatGloss) ·0.1 + clearcoatGloss ·0.001.
    Real a_g=(1-clearcoatGloss)*0.1+clearcoatGloss*0.001;
    //Dc = αg^2 −1/π*log(αg^2)*(1 + (αg^2 −1)*(hlz)^2)
    Real D_c=(a_g*a_g-1.0)/(c_PI*log(a_g*a_g)*(1+(a_g*a_g-1)*h_local.z*h_local.z));
    
    Vector3 win_local=to_local(frame, dir_in);
    Vector3 wout_local=to_local(frame, dir_out);
    //Λc(ω) =(√((1 + ((ωl.x·0.25)^2+(ωl.y·0.25)^2))/ωl.z^2) −1)/2
    Real lambda_win=(sqrt(1.0+(win_local.x*0.25*win_local.x*0.25+win_local.y*0.25*win_local.y*0.25)/(win_local.z*win_local.z))-1)/2.0;
    Real lambda_wout=(sqrt(1.0+(wout_local.x*0.25*wout_local.x*0.25+wout_local.y*0.25*wout_local.y*0.25)/(wout_local.z*wout_local.z))-1)/2.0;
    //Gc(ω) = 1/1 + Λc(ω)
    Real G_c_win=1.0/(1.0+lambda_win);
    Real G_c_wout=1.0/(1.0+lambda_wout);
    //Gc = Gc(ωin)Gc(ωout)
    Real G_c=G_c_win*G_c_wout;

    //fclearcoat = FcDcGc/4|n ·ωin|
    Real f_clearcoat=(F_c*D_c*G_c*n_dot_out)/(4.0*n_dot_in);
    return make_const_spectrum(f_clearcoat);
}

Real pdf_sample_bsdf_op::operator()(const DisneyClearcoat &bsdf) const {
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
    Real clearcoatGloss= eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    Vector3 h = normalize(dir_in+dir_out);
    Vector3 h_local=to_local(frame, h);

    Real n_dot_in=fabs(dot(frame.n, dir_in));
    Real h_dot_out=fabs(dot(h, dir_out));

    //αg = (1 −clearcoatGloss) ·0.1 + clearcoatGloss ·0.001.
    Real a_g=(1-clearcoatGloss)*0.1+clearcoatGloss*0.001;
    //Dc = αg^2 −1/π*log(αg^2)*(1 + (αg^2 −1)*(hlz)^2)
    Real D_c=(a_g*a_g-1.0)/(c_PI*log(a_g*a_g)*(1+(a_g*a_g-1)*h_local.z*h_local.z));

    Vector3 win_local=to_local(frame, dir_in);
    Real lambda_win=(sqrt(1.0+(win_local.x*0.25*win_local.x*0.25+win_local.y*0.25*win_local.y*0.25)/(win_local.z*win_local.z))-1)/2.0;
    
    //Gc(ω) = 1/1 + Λc(ω)
    Real G_c_win=1.0/(1.0+lambda_win);
    //pdf=D_c*G(win)/4|n ·ωin|
    return (D_c*G_c_win*h_dot_out)/(4.0*n_dot_in);
}

std::optional<BSDFSampleRecord>
        sample_bsdf_op::operator()(const DisneyClearcoat &bsdf) const {
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
    Real clearcoatGloss= eval(bsdf.clearcoat_gloss, vertex.uv, vertex.uv_screen_size, texture_pool);
    

    // Clearcoat uses fixed roughness 0.25 for sampling (same as G term)
    Real sampling_roughness = Real(0.25);
    Vector3 win_local=to_local(frame, dir_in);
    Vector3 local_micro_normal = sample_visible_normals(win_local, sampling_roughness, rnd_param_uv);
    Vector3 wout_local = 2.0 * dot(local_micro_normal, win_local) * local_micro_normal - win_local;
    
    Vector3 dir_out = to_world(frame, wout_local);

    if (dot(vertex.geometric_normal, dir_out) <= 0) {
        return {};
    }

    return BSDFSampleRecord{
        dir_out,
        Real(0),  
        Real(0.25)
    };
}

TextureSpectrum get_texture_op::operator()(const DisneyClearcoat &bsdf) const {
    return make_constant_spectrum_texture(make_zero_spectrum());
}
