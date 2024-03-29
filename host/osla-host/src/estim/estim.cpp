#include "estim.h"
#include "../mmio/mmio.h"
#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

/**
 * Delays destination. To purposefully insert a delay of D, set D_hat = D
 * 
 * @param D_hat estimated relative delay of source wrt destination
 * 
*/
void compensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat) {
    uint16_t dest_delay, src_delay;
    if (D_hat < 0) { // D < 0 -> prmbl early -> delay src
        dest_delay = 0;
        src_delay = -D_hat;
    }
    else { // D > 0 -> prmbl late -> delay dest
        dest_delay = D_hat;
        src_delay = 0;
    }
    wr_mem_cmd(tx_usrp, (src_delay_cmd << 32) | src_delay);
    wr_mem_cmd(tx_usrp, (dest_delay_cmd << 32) | dest_delay);
}

/**
 * @brief Upsamples a vector by duplicating each sample N times.
 *
 * This template function takes a vector of either arithmetic types or std::complex<double>
 * and an upsampling factor 'N'. It creates and returns a new vector where each element 
 * from the original vector is duplicated 'N' times.
 *
 * @tparam T The type of elements in the vector (arithmetic type or std::complex<double>).
 * @param input The original vector to be upsampled.
 * @param N The upsampling factor. Each sample in the original vector will be duplicated 'N' times.
 * @return A new vector representing the upsampled data.
 */
template <typename T>
std::vector<T> upsample(const std::vector<T>& input, int N) {
    static_assert(std::is_arithmetic<T>::value || std::is_same<T, std::complex<double>>::value,
                  "Unsupported type. Only arithmetic types or complex<double> are allowed.");

    std::vector<T> upsampled;

    for (const auto& value : input) {
        for (int i = 0; i < N; ++i) {
            upsampled.push_back(value);
        }
    }

    return upsampled;
}

// Explicit instantiation for types you plan to use
template std::vector<double> upsample(const std::vector<double>& input, int N);
template std::vector<std::complex<double>> upsample(const std::vector<std::complex<double>>& input, int N);