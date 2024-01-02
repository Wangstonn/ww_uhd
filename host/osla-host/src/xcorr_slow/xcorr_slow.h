#ifndef XCORR_SLOW_H
#define XCORR_SLOW_H

#include <vector>
#include <complex>

void xcorr_slow(const std::vector<std::complex<double>>& x, const std::vector<std::complex<double>>& y, std::vector<std::complex<double>>& r, std::vector<int>& lags);

#endif  // XCORR_SLOW_H
