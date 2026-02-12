#include "src/lajolla.h"
#include "src/microfacet.h"
#include <iostream>

int main() {
    // Test Fresnel at normal incidence for glass (eta=1.5)
    Real eta = 1.5;
    Real n_dot_i = 1.0; // normal incidence
    Real F = fresnel_dielectric(n_dot_i, eta);
    
    std::cout << "Fresnel at normal incidence (eta=1.5): " << F << std::endl;
    std::cout << "Should be around 0.04 (4% reflection)" << std::endl;
    std::cout << "Transmission probability: " << (1.0 - F) << std::endl;
    
    // Test at grazing angle
    n_dot_i = 0.1;
    F = fresnel_dielectric(n_dot_i, eta);
    std::cout << "\nFresnel at grazing angle: " << F << std::endl;
    std::cout << "Transmission probability: " << (1.0 - F) << std::endl;
    
    return 0;
}
