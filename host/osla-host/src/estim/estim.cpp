#include "estim.h"
#include "../mmio/mmio.h"
#include <vector>
#include <complex>
#include <numeric>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

const double prmbl_amp = (1 - std::pow(2, -15));

/**
 * @brief Estimates channel characteristics based on captured samples.
 * 
 * This function estimates channel coefficient h_hat and delay d_hat.
 * 
 * @param tx_usrp A pointer to the UHD USRP object for transmission.
 * @param D_test The test delay to be compensated.
 * @param rx_ch_sel_bits Selector bits for receiving channels.
 * @param tx_core_bits Selector bits for transmission core.
 * @param gpio_start_sel_bits Selector bits for GPIO start.
 * @param NCapSamps Number of samples to be captured.
 * @param var Variance of the captured samples.
 * @return ChParams struct containing d_hat and h_hat 
 */
ChParams ch_estim(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_test, const std::uint32_t rx_ch_sel_bits, const std::uint32_t tx_core_bits, const std::uint32_t gpio_start_sel_bits, const int& NCapSamps, const std::string& file) {
    compensateDelays(tx_usrp, D_test);

    //Configure runtime mode-------------------------------------------------------------------------------------------
    std::uint32_t mode_bits{0x0};
    mmio::start_tx(tx_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits);

    //Read data
    std::vector<std::complex<double>> cap_samps;
    mmio::read_sample_mem(tx_usrp, cap_samps, NCapSamps, ""); //dont write to file since itll be overwritten

    int N_w = static_cast<int>(cap_samps.size()); //number of captured samples

    // Estimate phase------------------------------------------------------------------------------
    // Load preamble
    std::ifstream if_file("preamble.mem"); // Open the file. Notice that this is the relative path from the executable location!!

    if (!if_file.is_open()) {
        std::cerr << "Error opening the preamble file!" << std::endl;
        ChParams ch_params;
        return ch_params;
    }

    // Read data from the file and store it as individual bits in a vector
    std::vector<int> preamble_bits;
    char bit;
    while (if_file >> bit) {
        preamble_bits.push_back(bit - '0'); // Convert character to integer (0 or 1)
    }

    if_file.close(); // Close the file

    // Calculate prmbl_samps
    std::vector<std::complex<double>> prmbl_samps;
    for (const auto& bit : preamble_bits) {
        std::complex<double> val = {2 * (bit - 0.5) * prmbl_amp,0};
        prmbl_samps.push_back(val);
    }

    prmbl_samps = upsample(prmbl_samps,32);
    int N_prmbl = static_cast<int>(prmbl_samps.size());

    // // Display the values calculated
    //std::cout << "N_prmbl: " << N_prmbl << std::endl;
    // std::cout << "prmbl_amp: " << prmbl_amp << std::endl;
    // std::cout << "prmbl_samps: ";
    // for (int i = 0; i < N_prmbl; ++i) {
    //     std::cout << prmbl_samps[i] << " ";
    // }
    // std::cout << std::endl;
    
    
    std::vector<std::complex<double>> r;
    std::vector<int> lags;
    xcorr_slow(prmbl_samps,cap_samps, r, lags); //prmbl samps dragged over cap_samps

    // Find the index of the maximum absolute value in vector r
    auto max_it = std::max_element(r.begin(), r.end(), [](const std::complex<double>& a, const std::complex<double>& b) {
        return std::abs(a) < std::abs(b);
    }); //Finds the iterator pointing to the max element
    int max_idx = std::distance(r.begin(), max_it); //Finds the index corresponding to that iterator

    // Calculate D_hat (lag at max_idx)
    int D_hat;
    D_hat = lags[max_idx];


    // // Output the cross-correlation values and corresponding lags
    // std::cout << "Cross-correlation result:" << std::endl;
    // for (size_t i = 0; i < r.size(); ++i) {
    //     std::cout << std::dec << "Lag: " << lags[i] << ", Correlation: " << r[i] << std::endl;
    // }
    

    // Our estimate depends on the number of samples captured in the capture window (of size N_w)
    int N_samps_cap = 0;
    if (D_hat > 0) {
        N_samps_cap = N_w - D_hat;
    } else if (D_hat < N_w - N_prmbl) {
        N_samps_cap = N_prmbl + D_hat;
    } else {
        N_samps_cap = N_w;
    }

    // Calculate h_hat, h_hat_mag, and phi_hat
    std::complex<double> h_hat;
    h_hat = r[max_idx] / (N_samps_cap * std::pow(prmbl_amp, 2));

    D_hat += D_test; //account for the test delay we inserted

    ChParams ch_params;
    ch_params.D_hat = D_hat;
    ch_params.h_hat = h_hat;
    return ch_params;


    //matches matlab
    // std::cout << std::dec;
    // std::cout << "Number of captured samples: " << N_w << std::endl;
    // std::cout << "D_test = " << D_test << ", ";
    // std::cout << "D_hat: " << D_hat << std::endl;
    // std::cout << "N_samps_cap: " << N_samps_cap << std::endl;
    // std::cout << "h_hat: " << h_hat << std::endl;
    // std::cout << "h_hat_mag: " << std::abs(h_hat) << std::endl;
    // std::cout << "phi_hat: " << std::arg(h_hat) << std::endl;
}

double EstimNoise(const uhd::usrp::multi_usrp::sptr tx_usrp, int NCapSamps){
    mmio::start_tx(tx_usrp, 0x0, 0x1, 0x0, 0x0); //Mode zero, only listen at src (no tx)

    //Read on chip acquired data and write to binary file to be parsed by matlab
    std::vector<std::complex<double>> cap_samps;
    mmio::read_sample_mem(tx_usrp, cap_samps, NCapSamps,"");

    // Estimate noise
    std::complex<double> sum = std::accumulate(std::begin(cap_samps), std::end(cap_samps), std::complex<double>{0,0});
    //long unsigned int size->double is a narrowing but hopefully our vectors dont have this size
    std::complex<double> mu =  sum / std::complex<double>{static_cast<double>(cap_samps.size()),0};

    double accum = 0;
    std::for_each(std::begin(cap_samps), std::end(cap_samps), [&](const std::complex<double> d) {
        accum += std::pow(std::abs(d - mu),2);
    });

    double var = accum / (cap_samps.size()-1);
    return var;
}


double calcSNR(const std::complex<double>& h_hat, const double var)
{
    double SNR = 10*std::log10(std::pow(prmbl_amp*std::abs(h_hat),2)/(var*2));
    return SNR;
}

double calcEsN0(const std::complex<double>& h_hat, const int osr, const double var)
{
    double EsN0 = 10*std::log10(std::pow(prmbl_amp*std::abs(h_hat),2)*osr/(var*2));
    return EsN0;
}


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
    mmio::wr_mem_cmd(tx_usrp, (mmio::src_delay_cmd << 32) | src_delay);
    mmio::wr_mem_cmd(tx_usrp, (mmio::dest_delay_cmd << 32) | dest_delay);
}


const int kEqFrac = 13;

/**
 * Sets the digital flatfading equalization parameters for the destination 
 * 
 * h_hat magnitude can range from 0.5 to 2. This should be applied after analog gain/tx amp is set.
 * 
 * @param h_hat The channel estimate in complex double format.
 * @param tx_usrp A shared pointer to the UHD multi_usrp object for transmitting.
 */
void PhaseEq(uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double>& h_hat) {
    std::complex<double> reciprocal_h_hat = 1.0 / h_hat;
    
    std::cout << reciprocal_h_hat << std::endl;
    double dest_ch_eq_re = std::real(reciprocal_h_hat)*std::pow(2, kEqFrac);
    double dest_ch_eq_im = -std::imag(reciprocal_h_hat)*std::pow(2, kEqFrac); //because dest expects the phase of the channel, not the reciprocal


    //covert to bit command
    int16_t dest_ch_eq_re_int16 = static_cast<int16_t>(std::round(dest_ch_eq_re));
    int16_t dest_ch_eq_im_int16 = static_cast<int16_t>(std::round(dest_ch_eq_im));

    // Apply saturation check
    if (dest_ch_eq_re > INT16_MAX || dest_ch_eq_re < INT16_MIN) {
        dest_ch_eq_re_int16 = (dest_ch_eq_re > INT16_MAX) ? INT16_MAX : INT16_MIN;
        std::cout << "WARNING: dest_ch_eq saturated" << std::endl;
    }
    if (dest_ch_eq_im > INT16_MAX || dest_ch_eq_im < INT16_MIN) {
        dest_ch_eq_im_int16 = (dest_ch_eq_im > INT16_MAX) ? INT16_MAX : INT16_MIN;
        std::cout << "WARNING: dest_ch_eq saturated" << std::endl;
    }

    mmio::WrMmio(tx_usrp, mmio::kDestChEqReAddr, static_cast<uint16_t>(dest_ch_eq_re_int16));
    mmio::WrMmio(tx_usrp, mmio::kDestChEqImAddr, static_cast<uint16_t>(dest_ch_eq_im_int16));
}

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