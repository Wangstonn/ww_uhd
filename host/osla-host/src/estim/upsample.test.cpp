#include <iostream>
#include <vector>
#include <complex>
#include <type_traits>
#include "estim.h"


int main() {
    // Example vector of doubles
    std::vector<double> originalVectorDoubles = {1.0, 2.0, 3.0};

    // Example vector of complex doubles
    std::vector<std::complex<double>> originalVectorComplexDoubles = {
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0}
    };

    // Upsample doubles by duplicating each sample 2 times
    int N = 2;
    std::vector<double> upsampledVectorDoubles = upsample(originalVectorDoubles, N);

    // Upsample complex doubles by duplicating each sample 3 times
    N = 3;
    std::vector<std::complex<double>> upsampledVectorComplexDoubles = upsample(originalVectorComplexDoubles, N);

    // Output the original and upsampled vectors
    std::cout << "Original Vector (Doubles): ";
    for (const auto& value : originalVectorDoubles) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "Upsampled Vector (Doubles): ";
    for (const auto& value : upsampledVectorDoubles) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "Original Vector (Complex Doubles): ";
    for (const auto& value : originalVectorComplexDoubles) {
        std::cout << "(" << value.real() << ", " << value.imag() << ") ";
    }
    std::cout << std::endl;

    std::cout << "Upsampled Vector (Complex Doubles): ";
    for (const auto& value : upsampledVectorComplexDoubles) {
        std::cout << "(" << value.real() << ", " << value.imag() << ") ";
    }
    std::cout << std::endl;

    return 0;
}
