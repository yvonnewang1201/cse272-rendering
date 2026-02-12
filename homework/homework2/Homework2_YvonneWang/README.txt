CSE 272 Homework 2 - Volumetric Path Tracing
============================================

Name: Yvonne Wang
Student ID: A69042268
Email: yvw001@ucsd.edu

Required Implementations
------------------------
All six volumetric path tracers have been implemented in:
  code/vol_path_tracing.h

1. vol_path_tracing_1: Single absorption homogeneous volume
2. vol_path_tracing_2: Single scattering with NEE + Equiangular Sampling MIS
3. vol_path_tracing_3: Multiple scattering homogeneous
4. vol_path_tracing_4: Multiple scattering with NEE
5. vol_path_tracing_5: Full volumetric path tracer with MIS and surface lighting
6. vol_path_tracing_6: Heterogeneous volumes with delta tracking

Required Renders (in images/ folder)
------------------------------------
- volpath_test1.exr: vol_path_tracing_1 test
- volpath_test2.exr: vol_path_tracing_2 test
- volpath_test3.exr: vol_path_tracing_3 test
- volpath_test4.exr: vol_path_tracing_4 test
- volpath_test4_2.exr: vol_path_tracing_4 dense volume test
- volpath_test5.exr: vol_path_tracing_5 test
- volpath_test5_2.exr: vol_path_tracing_5 dielectric interface test
- volpath_test6.exr: vol_path_tracing_6 chromatic test
- vol_cbox.exr: Cornell box with participating media
- vol_cbox_teapot.exr: Cornell box with teapot in fog
- hetvol.exr: Heterogeneous volume (bunny smoke)
- hetvol_colored.exr: Colored heterogeneous volume

================================================================================
BONUS 1: Equiangular Sampling (35%)
================================================================================

Implementation: code/vol_path_tracing.h (lines 1-117 for helper functions)

Equiangular sampling is implemented as described in:
"Importance Sampling Techniques for Path Tracing in Participating Media"
by Christopher Kulla and Marcos Fajardo, EGSR 2012

Key functions:
- sample_equiangular(): Samples distance using equiangular distribution
- pdf_equiangular(): Computes PDF for equiangular sampling

The sampling strategy uses one-sample MIS between transmittance sampling
and equiangular sampling with 50/50 probability, using the balance heuristic.

Applied to: vol_path_tracing_2 (single scattering with NEE)

Comparison renders:
- volpath_test2_with_equiangular.exr: With equiangular MIS
- volpath_test2_no_equiangular.exr: Transmittance sampling only

The equiangular sampling reduces variance especially when there is a point
or small area light source inside participating media, as it importance
samples the geometric term 1/|p - light|^2 along the ray.

================================================================================
BONUS 2: Custom Scene (20%)
================================================================================

Scene files (in code/ folder):
- jellyfish_sea.xml: Main jellyfish scene (uses vol_path_tracing_6)
- jellyfish_skin.obj, jellyfish_arm.obj, jellyfish_core.obj: Mesh files
- jellyfish_equiangular_test.xml: Simplified scene for equiangular comparison

Rendered outputs:
- jellyfish_bonus.exr: Full jellyfish scene with multiple scattering

Description: An underwater jellyfish scene demonstrating volumetric path
tracing capabilities:
- Ocean medium with wavelength-dependent absorption and scattering
- Translucent jellyfish body with internal scattering medium
- Bioluminescent cyan glow from the jellyfish core
- Sunlight filtering from above

================================================================================
QUESTION ANSWERS
================================================================================

Q2.1 Single monochromatic absorption-only homogeneous volume (4 pts)
---------------------------------------------------------------------
Question: Change the absorption parameters to zero in volpath_test1.xml.
What do you see? Why?

Answer: When you change the absorption parameters to zero in the scene file,
you will see the emissive sphere rendered with its full original brightness
and color, appearing exactly as it would if it were in a vacuum with no
participating medium at all. This occurs because this specific renderer
models an absorption-only volume where the light reaching the camera is
calculated using the Beer-Lambert law, which multiplies the emitted radiance
by a transmittance factor of e^(-sigma_a * distance). When sigma_a is set to
zero, the exponent becomes zero, and since e^0 = 1, the transmittance becomes
perfectly clear. Consequently, the light travels from the object to the camera
without undergoing any attenuation or energy loss.

Q2.2 (4 pts)
------------
Question: If you were tasked to modify the pseudo code above to add volume
emission, how would you do it?

Answer: To add volume emission, I would modify the function to account for
the light emitted by the medium itself along the ray path, in addition to
the light coming from the background surface. Since the current renderer
assumes a homogeneous medium with constant absorption and emission coefficients,
I can compute this contribution analytically without needing Monte Carlo
sampling. I would calculate the accumulated volumetric emission by integrating
the constant emission along the ray distance, which results in a term equal to
the volumetric emission intensity multiplied by (1 - transmittance) / sigma_a.
I would then add this calculated volumetric radiance to the existing attenuated
surface radiance.

Q3.1 (2 pts)
------------
Question: How did we get from p(t) proportional to exp(-sigma_t*t) to
p(t) = sigma_t * exp(-sigma_t*t)?

Answer: To get from the proportional relationship to the equality, we need
to normalize the PDF so that it integrates to 1 over [0, infinity). The
integral of exp(-sigma_t * t) from 0 to infinity equals 1/sigma_t. To ensure
the total probability equals 1, we multiply by sigma_t, giving us the final
PDF p(t) = sigma_t * exp(-sigma_t * t).

Q3.2 (2 pts)
------------
Question: How was Equation (13) incorporated into the pseudo code? Why?

Answer: Equation 13 is incorporated in the else block, which executes when
the sampled distance is greater than t_hit (distance to surface). The
trans_pdf is set to exp(-sigma_t * t_hit), which is the probability that
light travels the entire distance to the surface without scattering. In
Monte Carlo integration, the contribution must be divided by the probability
of obtaining that sample, ensuring correct weighting.

Q3.3 (2 pts)
------------
Question: Play with sigma_s and sigma_a parameters. How do they affect the image?

Answer: Increasing sigma_a (absorption) makes the volume darker because more
light energy is absorbed by the medium. Increasing sigma_s (scattering) makes
the volume appear brighter with a glowing halo effect around light sources,
while simultaneously making background objects harder to see. This is because
scattering redirects light toward the camera (in-scattering) while also
bouncing light away from the camera path (out-scattering). Both parameters
increase the extinction coefficient, making the medium more opaque.

Q3.4 (2 pts)
------------
Question: What does the g parameter in Henyey-Greenstein mean? How does it
change appearance?

Answer: The parameter g is the asymmetry parameter (mean cosine of scattering
angle), controlling whether light scatters forward, backward, or uniformly.
When g > 0 (forward scattering), the volume appears brighter when looking
toward the light source, creating strong halos. When g < 0 (backward scattering),
the volume appears brighter when the light is behind the camera. When g = 0,
scattering is isotropic, resulting in uniform appearance.

Q4.1 (2.667 pts)
----------------
Question: How do sigma_s, sigma_a and max_depth affect the image?

Answer: Increasing sigma_a makes volumes darker due to absorption. Increasing
sigma_s makes volumes more opaque and "cloudy" - with high scattering albedo,
it creates bright, cloud-like appearance. Yes, different values affect
max_depth requirements: high sigma_s with low sigma_a allows light to bounce
many times, requiring higher max_depth for accurate simulation. If max_depth
is too low, the image will be artificially darker due to premature path
termination.

Q4.2 (2.667 pts)
----------------
Question: How does g parameter affect appearance in multiple scattering?

Answer: In multiple scattering, g controls how deep light penetrates. Positive
g (forward scattering) allows deeper penetration and creates concentrated glows
around lights, making the medium feel softer and more translucent. Negative g
(backward scattering) prevents deep penetration, making the medium appear more
opaque and reflective on the lit side.

Q4.3 (2.666 pts)
----------------
Question: Propose a phase function design.

Answer: I propose a dual-lobed phase function by linearly blending two
Henyey-Greenstein functions to capture complex scattering like clouds or dust.
The shape would resemble a dumbbell with forward and backward lobes. Parameters:
(1) forward asymmetry for the forward peak sharpness, (2) backward asymmetry
for backward reflection width, (3) blending weight 0-1 to control energy
distribution between lobes.

Q5.1 (2.667 pts)
----------------
Question: When is NEE more efficient than phase function sampling?

Answer: NEE is more efficient with small, intense light sources or optically
thin media, as it explicitly connects scattering events to lights. In our
test scenes (like Cornell Box) with small area lights and isotropic scattering,
NEE is significantly more efficient because the probability of randomly
hitting the light via phase sampling is very low, causing high variance.

Q5.2 (2.667 pts)
----------------
Question: How does dense volume compare to Lambertian material?

Answer: A dense volume appears remarkably similar to a Lambertian material
because the high scattering coefficient creates an extremely short mean free
path. Light penetrates only shallowly before multiple scattering randomizes
its direction, mimicking diffuse reflection. However, the volumetric approach
allows subsurface scattering, creating softer edges compared to the hard
reflection of pure Lambertian surfaces.

Q5.3 (2.666 pts)
----------------
Question: Why did Kajiya predict "all rendering will be volume rendering"?
Why hasn't it happened?

Answer: Kajiya recognized that volume rendering offers a unified theory - real
matter is particles with varying densities, and surface rendering is just a
limiting case where density becomes infinite at boundaries. However, this
transition hasn't happened due to computational cost. Treating hard surfaces
as extremely dense volumes creates "stiff" problems requiring microscopic
step sizes, making boundary-representation methods significantly faster for
opaque objects.

Q6.1 (4 pts)
------------
Question: How does index of refraction affect appearance?

Answer: Higher IOR increases Fresnel reflectance, making the surface more
reflective and preventing environmental light from entering. It also causes
stronger ray bending (Snell's Law), acting as a stronger lens that warps the
internal volume's appearance and changes light path lengths through the medium.

Q6.2 (4 pts)
------------
Question: Difference between blue medium inside glass vs just blue glass?

Answer: Blue medium inside creates realistic depth-dependent color - thicker
parts look deeper blue due to absorption along the path length. It also
enables internal scattering for cloudy/hazy appearance. In contrast, blue
glass surface color creates a hollow colored shell where the tint is constant
regardless of the volume inside, failing to capture the absorbing nature
of liquid or depth-dependent color gradients.

Q7.1 (2.667 pts)
----------------
Question: When is null scattering efficient/inefficient? How to improve?

Answer: Null scattering is inefficient with sparse or highly variable density
distributions where small high-density regions are surrounded by empty space.
The global majorant must match the highest peak, causing tiny steps even in
empty regions. It's most efficient with uniform density. To improve, use
spatially adaptive structures (macro-grid, octree, KD-tree) storing local
majorants, allowing large steps through empty space.

Q7.2 (2.667 pts)
----------------
Question: How to make null-scattering work for emissive volumes?

Answer: Accumulate emission during the delta tracking loop. At each step,
evaluate volumetric emission at the current position, weight it by current
path transmittance, and add to accumulated radiance. This effectively
integrates the source term while sampling the next scattering distance.

Q7.3 (2.666 pts)
----------------
Question: Why is unbiased important? Would biased but faster be sensible?

Answer: Unbiased solutions guarantee expected pixel values match the correct
result, converging to ground truth with more samples. Yes, biased but faster
solutions are sensible for real-time applications where eliminating noise
quickly matters more than perfect accuracy. Implementation: use deterministic
ray marching with fixed step sizes instead of stochastic sampling, trading
discretization error (bias) for noise-free results.

================================================================================
Build Instructions
================================================================================

cd lajolla_public
mkdir build && cd build
cmake ..
make -j

Running the renderer:
./lajolla <scene.xml>
