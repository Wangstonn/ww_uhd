#include <iostream>
#include <vector>
#include <complex>
#include "xcorr_slow.h"

//unit test for xcorr_slow
//g++ -o test xcorr_slow.test.cpp xcorr_slow.cpp

int main() {
    // Example complex arrays x and y
    std::vector<std::complex<double>> x = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}, {4.0, 5.0}, {5.0, 6.0}};
    std::vector<std::complex<double>> y = {{2.0, 1.0}, {3.0, 2.0}, {4.0, 3.0}};

    std::vector<std::complex<double>> result;
    std::vector<int> lags;

    // Compute cross-correlation for complex arrays
    xcorr_slow(x, y, result, lags);

    // Output the cross-correlation values and corresponding lags
    std::cout << "Cross-correlation result:" << std::endl;
    for (size_t i = 0; i < result.size(); ++i) {
        std::cout << "Lag: " << lags[i] << ", Correlation: " << result[i] << std::endl;
    }

    return 0;
}