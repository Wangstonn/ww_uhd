#include <vector>
#include <complex>
#include "xcorr_slow.h"

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

