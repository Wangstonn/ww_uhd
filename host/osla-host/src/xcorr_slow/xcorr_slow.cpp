#include <vector>
#include <complex>
#include "xcorr_slow.h"

/**
 * @brief Computes the cross-correlation of two complex-valued vectors.
 *
 * The function calculates the cross-correlation between two input vectors, `x` and `y`, and stores the results in the output vectors `r` and `lags`.
 * Cross-correlation is a measure of similarity between two signals as a function of the time lag applied to one of them.
 *
 * @param x Input vector representing the first complex signal. It is conjugated and dragged over y
 * @param y Input vector representing the second complex signal.
 * @param r Output vector containing the cross-correlation values.
 * @param lags Output vector containing the corresponding time lags for each cross-correlation value.
 *
 * @details The function uses a nested loop to iterate over all possible time lags within the range [-n+1, m-1].
 *          For each lag, it calculates the cross-correlation sum by multiplying the complex conjugate of `x`
 *          with the corresponding element in `y`. The results are stored in the output vectors `r` and `lags`.
 *
 * @note The input vectors `x` and `y` must be of complex type, and the output vectors `r` and `lags` are modified in place.
 *
 */
void xcorr_slow(const std::vector<std::complex<double>>& x, const std::vector<std::complex<double>>& y, std::vector<std::complex<double>>& r, std::vector<int>& lags) {
    int n = static_cast<int>(x.size());
    int m = static_cast<int>(y.size());

    int maxLag = n + m - 1; // Maximum lag for cross-correlation

    r.resize(maxLag, std::complex<double>(0.0, 0.0)); // Initialize result vector with zeros
    lags.resize(maxLag, 0); // Initialize lags vector

    for (int lag = -n + 1; lag < m; ++lag) {
        std::complex<double> sum(0.0, 0.0);
        int startIdxX = std::max(0, -lag);
        int endIdxX = std::min(n - 1, m - 1 - lag);

        for (int i = startIdxX; i <= endIdxX; ++i) {
            int j = i + lag;
            sum += std::conj(x[i]) * y[j];
        }

        int rIdx = lag + n - 1;
        r[rIdx] = sum;
        lags[rIdx] = lag;
    }
}

