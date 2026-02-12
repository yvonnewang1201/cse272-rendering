Spectrum eval_op::operator()(const DisneyDiffuse &bsdf) const {
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
    Real subsurface = eval(bsdf.subsurface, vertex.uv, vertex.uv_screen_size, texture_pool);
    
    
    Vector3 h = normalize(dir_in+dir_out);

    Real n_dot_out=fabs(dot(frame.n, dir_out));
    Real n_dot_in=fabs(dot(frame.n, dir_in));
    Real h_dot_out=fabs(dot(h, dir_out));

    //F_D90= 1/2 + 2 ·roughness ·|h ·ωout|^2
    Real F_D90=0.5+2.0*roughness*h_dot_out*h_dot_out;
    //F_D(ω)= (1 + (FD90 −1)(1 −|n ·ω|)^5)
    Real F_D_in=1.0+(F_D90-1)*pow((1.0-n_dot_in), 5.0);
    Real F_D_out=1.0+(F_D90-1)*pow((1.0-n_dot_out), 5.0);

    //fbaseDiffuse = baseColor/π * FD(ωin) * FD(ωout)|n ·ωout|
    Spectrum f_base=(base_color/c_PI)*F_D_in*F_D_out*n_dot_out;
    
    //FSS90 = roughness ·|h ·ωout|^2
    Real F_SS90=roughness*h_dot_out*h_dot_out;

    //FSS(ω) = (1 + (FSS90 −1)(1 −|n ·ω|)^5)
    Real F_SS_in=(1.0+(F_SS90-1.0)*pow(1.0-(n_dot_in), 5));
    Real F_SS_out=(1.0+(F_SS90-1.0)*pow(1.0-(n_dot_out), 5));

    //fsubsurface = 1.25·baseColor/π · [FSS(ωin)·FSS(ωout) · (1/(|n·ωin| + |n·ωout|) - 0.5) + 0.5] · |n·ωout|
    Spectrum f_subsurface = 1.25*(base_color / c_PI)*(F_SS_in * F_SS_out*((1.0/(n_dot_in+n_dot_out))-0.5)+0.5) * n_dot_out;
    
    //f_diffuse=(1 − subsurface) · fbaseDiffuse + subsurface * fsubsurface
    Spectrum f_diffuse=(1.0-subsurface)*f_base+subsurface*f_subsurface;

    return f_diffuse;
}




Real pdf_sample_bsdf_op::operator()(const DisneyDiffuse &bsdf) const {
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
    return fmax(dot(frame.n, dir_out), Real(0)) / c_PI;
}

std::optional<BSDFSampleRecord> sample_bsdf_op::operator()(const DisneyDiffuse &bsdf) const {
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
    return BSDFSampleRecord{
        to_world(frame, sample_cos_hemisphere(rnd_param_uv)),
        Real(0) /* eta */, Real(1) /* roughness */};
}

TextureSpectrum get_texture_op::operator()(const DisneyDiffuse &bsdf) const {
    return bsdf.base_color;
}
