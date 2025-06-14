#include <uhd/usrp/multi_usrp.hpp>
#include <type_traits>
#include <boost/math/distributions/chi_squared.hpp> // For chi_squared distribution
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include "../mmio/mmio.h"
#include "estim.h"

namespace estim {

bool send_message(int sock, bool event, double intf_rss_dbm, double target_intf_rss_dbm)
{
    MSG_t msg = {event, intf_rss_dbm, target_intf_rss_dbm};

    int sent = send(sock, &msg, sizeof(msg), 0);
    if (sent < 0) {
        perror("send");
        close(sock);
        return false;
    }
    return true;
}

int connectToServerSock()
{
    int sock;
    struct sockaddr_in server_addr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(12345);
    server_addr.sin_addr.s_addr = inet_addr("141.213.15.219"); // <- hsklabu01 AA4.eecs.umich.edu 141.213.15.85

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("connect");
        close(sock);
        return -1;
    }
    return sock;
}
// int main(int argc, char *argv[]) {
//   int sock = connectToServerSock();
//   if (sock < 0) {
//     return 1;
//   }
//   send_message(sock, 1.0, 2.0, true);
//   close(sock);
//   return 0;
// }

const double prmbl_amp = (1 - std::pow(2, -15));

/**
 * Estimates the number of captured samples given the window size, preamble size, and
 * delay.
 *
 * @param N_w The number of captured samples in the window.
 * @param N_prmbl The number of preamble samples.
 * @param D_hat The estimated delay.
 * @return The number of captured samples based on the window size, preamble size, and
 * delay.
 *
 * This function estimates the number of samples captured (N_samps) by comparing the
 * number of captured samples in the window (N_w) with the number of preamble samples
 * (N_prmbl) and adjusting for the estimated delay (D_hat). The calculation varies
 * depending on whether N_w is greater than or less than N_prmbl, and further adjustments
 * are made based on the value of D_hat.
 */
int EstimNSampCap(const int N_w, const int N_prmbl, const int D_hat)
{
    int N_samps;
    if (N_w > N_prmbl) {
        if (D_hat < 0)
            N_samps = std::max(0, N_prmbl + D_hat);
        else if (D_hat < (N_w - N_prmbl))
            N_samps = N_prmbl;
        else
            N_samps = std::max(0, N_prmbl - (D_hat - (N_w - N_prmbl)));
    } else {
        if (D_hat < 0)
            N_samps = std::max(0, N_w + D_hat);
        else if (D_hat < (N_prmbl - N_w))
            N_samps = N_w;
        else
            N_samps = std::max(0, N_w - (D_hat - (N_prmbl - N_w)));
    }

    return N_samps;
}

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
ChParams ChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int D_test,
    const std::uint32_t rx_ch_sel_bits,
    const std::uint32_t tx_core_bits,
    const std::uint32_t gpio_start_sel_bits,
    const int& NCapSamps,
    const std::string& file)
{
    CompensateDelays(tx_usrp, D_test);

    // Configure runtime
    // mode-------------------------------------------------------------------------------------------
    mmio::WrMmio(tx_usrp, mmio::kDestChipCapEn, 0x0); // capture samples

    std::uint32_t mode_bits{0x0}; // sync mode
    mmio::StartTx(
        tx_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits, 0x0, 0x0);

    // Typically, preamble is so fast no delay is needed
    while (true) {
        // Run and check received pkt
        mmio::ClearAddrBuffer(tx_usrp);
        if ((mmio::RdMmio(tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)
            == mmio::kCapMaxNumSamps - 1) // Dest has finished recording
            break;
    }
    mmio::ClearAddrBuffer(tx_usrp);

    // Read data
    std::vector<std::complex<double>> cap_samps =
        mmio::ReadSampleMem(tx_usrp, 0b1, NCapSamps, file);
    int N_w = static_cast<int>(cap_samps.size()); // number of captured samples

    std::vector<std::complex<double>> rx_if(N_w); // downconverted rx
    const double pi = std::acos(-1);
    for (int n = 0; n < N_w; ++n) {
        rx_if[n] = cap_samps[n]
                   * std::exp(std::complex<double>(0,
                       -2 * pi / estim::kFwOsr
                           * n)); // The phase measurement will be off because the two
                                  // sinusoids are not synced yet
        // std::cout << "if_tone: " <<  std::exp(std::complex<double>(0,
        // 2*pi/estim::kFwOsr * n )) << std::endl;
    }

    // Estimate
    // delay------------------------------------------------------------------------------
    // To estimate the phase, first need to sync the digital if tones. To do this, need to
    // find the delay

    // Load preamble
    std::ifstream if_file("preamble.mem"); // Open the file. Notice that this is the
                                           // relative path from the executable location!!
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
        std::complex<double> val = {2 * (bit - 0.5), 0};
        prmbl_samps.push_back(val);
    }
    prmbl_samps = Upsample(prmbl_samps, 32); // upsample corresponding to the preamble osr
    int N_prmbl = static_cast<int>(prmbl_samps.size());

    // // Display the values calculated
    // std::cout << "N_prmbl: " << N_prmbl << std::endl;
    // std::cout << "prmbl_amp: " << prmbl_amp << std::endl;
    // std::cout << "prmbl_samps: ";
    // for (int i = 0; i < N_prmbl; ++i) {
    //     std::cout << prmbl_samps[i] << " ";
    // }
    // std::cout << std::endl;

    std::vector<std::complex<double>> r;
    std::vector<int> lags;
    XcorrSlow(prmbl_samps, rx_if, r, lags); // prmbl samps dragged over cap_samps

    // Find the index of the maximum absolute value in vector r
    auto max_it = std::max_element(r.begin(),
        r.end(),
        [](const std::complex<double>& a, const std::complex<double>& b) {
            return std::abs(a) < std::abs(b);
        }); // Finds the iterator pointing to the max element
    int max_idx = std::distance(
        r.begin(), max_it); // Finds the index corresponding to that iterator

    // Calculate D_hat (lag at max_idx)
    int D_hat;
    D_hat = lags[max_idx];

    // // Output the cross-correlation values and corresponding lags
    // std::cout << "Cross-correlation result:" << std::endl;
    // for (size_t i = 0; i < r.size(); ++i) {
    //     std::cout << std::dec << "Lag: " << lags[i] << ", Correlation: " << r[i] <<
    //     std::endl;
    // }
    // // Our estimate depends on the number of samples captured in the capture window (of
    // size N_w) int N_prmbl_samps_cap = 0; if (D_hat > 0) {
    //     N_prmbl_samps_cap = N_w - D_hat;
    // } else if (D_hat < N_w - N_prmbl) {
    //     N_prmbl_samps_cap = N_prmbl + D_hat;
    // } else {
    //     N_prmbl_samps_cap = std::min(N_w,N_prmbl);
    // }

    int N_prmbl_samps_cap = EstimNSampCap(N_w,
        N_prmbl,
        D_hat); // Find how many samples were captured for coefficient estimation
    if (N_prmbl_samps_cap <= 0) {
        std::cerr << "Error: No samples captured" << std::endl;
    }

    // Calculate h_hat, h_hat_mag, and phi_hat
    std::complex<double> h_hat;
    h_hat = std::exp(std::complex<double>(0, 2 * pi / estim::kFwOsr * D_hat)) * r[max_idx]
            / double(N_prmbl_samps_cap);
    D_hat += D_test; // account for the test delay we inserted. The actual D_hat is 4 less
                     // than this measured value

    ChParams ch_params;
    ch_params.D_hat = D_hat;
    ch_params.h_hat = h_hat;
    return ch_params;

    // matches matlab
    //  std::cout << std::dec;
    //  std::cout << "Number of captured samples: " << N_w << std::endl;
    //  std::cout << "D_test = " << D_test << ", ";
    //  std::cout << "D_hat: " << D_hat << std::endl;
    //  std::cout << "N_samps_cap: " << N_samps_cap << std::endl;
    //  std::cout << "h_hat: " << h_hat << std::endl;
    //  std::cout << "h_hat_mag: " << std::abs(h_hat) << std::endl;
    //  std::cout << "phi_hat: " << std::arg(h_hat) << std::endl;
}

ChParams FbChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int D_test,
    const std::uint32_t rx_ch_sel_bits,
    const std::uint32_t tx_core_bits,
    const std::uint32_t gpio_start_sel_bits,
    const int& NCapSamps,
    const std::string& file)
{
    auto ch_params = estim::ChEstim(
        tx_usrp, D_test, 0b01, 0b10, gpio_start_sel_bits, pow(2, 15), file);
    ch_params.h_hat =
        ch_params.h_hat
        * std::pow(
            2, -6); // sample capture uses the ADC frac at destination. correct for source
    return ch_params;
}


/**
 * @brief Estimates channel characteristics based on captured samples. For p2p channels
 *
 * This function estimates channel coefficient h_hat and delay d_hat.
 *
 * @param src_tx_usrp A pointer to the UHD USRP object for transmission.
 * @param D_test The test delay to be compensated.
 * @param NCapSamps Number of samples to be captured.
 * @param is_forward Whether to perform forward or feedback channel estimation.
 * @return ChParams struct containing d_hat and h_hat
 */
ChParams P2PChEstim(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int D_test,
    const int& NCapSamps,
    const bool is_forward,
    const uint32_t start_sync,
    const bool skip_rst,
    const std::string& file)
{
    uhd::usrp::multi_usrp::sptr tx_usrp, rx_usrp;
    std::uint32_t gpio_start_sel_bits;

    // Feedback vs Forward estimation
    if (is_forward) {
        tx_usrp             = src_tx_usrp;
        rx_usrp             = dest_tx_usrp;
        gpio_start_sel_bits = estim::kFwdGpioStartSelBits;

    } else { // In feedback mode, source becomes dest, dest becomes source, dest becomes
             // gpio sender, source becomes gpio receiver
        tx_usrp             = dest_tx_usrp;
        rx_usrp             = src_tx_usrp;
        gpio_start_sel_bits = estim::kFbGpioStartSelBits;
    }

    std::uint32_t mode_bits{0x0};

    // Configure runtime
    // mode-------------------------------------------------------------------------------------------
    estim::P2PCompensateDelays(tx_usrp, rx_usrp, D_test);
    mmio::WrMmio(
        rx_usrp, mmio::kDestChipCapEn, 0x0); // capture samps for preamble analysis

    mmio::P2PStartTxRx(
        tx_usrp, rx_usrp, mode_bits, gpio_start_sel_bits, 0x0, start_sync, 0x0, skip_rst);

    int current_ctr;
    // If rst is skipped, that means we are locked and wait for the next sync window.
    // Else, just start normally
    if (start_sync) {
        if (skip_rst) {
            current_ctr = mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF;
            while (true) {
                mmio::ClearAddrBuffer(dest_tx_usrp);
                if ((mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF)
                    != current_ctr) {
                    break;
                }
            }
        } else
            current_ctr = 0;
    }
    current_ctr++;

    // Wait for sample capture to finish
    while (true) {
        // Run and check received pkt
        mmio::ClearAddrBuffer(rx_usrp);
        if ((mmio::RdMmio(rx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)
            == mmio::kCapMaxNumSamps - 1) // Dest has finished recording
            break;
    }
    mmio::ClearAddrBuffer(rx_usrp);

    // Read data
    std::vector<std::complex<double>> cap_samps =
        mmio::ReadSampleMem(rx_usrp, 0b1, NCapSamps, file);

    // make sure that sample read wasnt corrupted by the next run
    if (start_sync) {
        int post_read_ctr = mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF;
        if (current_ctr != post_read_ctr) {
            std::cout
                << "Error: Sync Lock Ctr advanced before samples were done reading! "
                   "Increase start period or decrease number of samples captured. \n"
                << std::dec << "Pre read lock idx: " << current_ctr << std::endl
                << "Post read lock idx: " << post_read_ctr << std::endl;
        }
    }

    int N_w = static_cast<int>(cap_samps.size()); // number of captured samples

    std::vector<std::complex<double>> rx_if(N_w); // downconverted rx
    const double pi = std::acos(-1);
    for (int n = 0; n < N_w; ++n) {
        rx_if[n] = cap_samps[n]
                   * std::exp(std::complex<double>(0,
                       -2 * pi / estim::kFwOsr
                           * n)); // The phase measurement will be off because the two
                                  // sinusoids are not synced yet
        // std::cout << "if_tone: " <<  std::exp(std::complex<double>(0,
        // 2*pi/estim::kFwOsr * n )) << std::endl;
    }

    // Estimate
    // delay------------------------------------------------------------------------------
    // To estimate the phase, first need to sync the digital if tones. To do this, need to
    // find the delay
    // Load preamble
    std::ifstream if_file("preamble.mem"); // Open the file. Notice that this is the
                                           // relative path from the executable location!!
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
        std::complex<double> val = {2 * (bit - 0.5), 0};
        prmbl_samps.push_back(val);
    }
    prmbl_samps = Upsample(prmbl_samps, 32); // upsample corresponding to the preamble osr
    int N_prmbl = static_cast<int>(prmbl_samps.size());

    std::vector<std::complex<double>> r;
    std::vector<int> lags;
    XcorrSlow(prmbl_samps, rx_if, r, lags); // prmbl samps dragged over cap_samps

    // Find the index of the maximum absolute value in vector r
    auto max_it = std::max_element(r.begin(),
        r.end(),
        [](const std::complex<double>& a, const std::complex<double>& b) {
            return std::abs(a) < std::abs(b);
        }); // Finds the iterator pointing to the max element
    int max_idx = std::distance(
        r.begin(), max_it); // Finds the index corresponding to that iterator

    // Calculate D_hat (lag at max_idx)
    int D_hat;
    D_hat = lags[max_idx];

    int N_prmbl_samps_cap = EstimNSampCap(N_w,
        N_prmbl,
        D_hat); // Find how many samples were captured for coefficient estimation
    if (N_prmbl_samps_cap <= 0) {
        std::cerr << "Error: No preamble samples captured in the window" << std::endl;
    }

    // Calculate h_hat, h_hat_mag, and phi_hat
    std::complex<double> h_hat;
    h_hat = std::exp(std::complex<double>(0, 2 * pi / estim::kFwOsr * D_hat)) * r[max_idx]
            / double(N_prmbl_samps_cap);
    if (!is_forward) {
        h_hat *= std::pow(2, -6);
    }
    D_hat += D_test; // account for the test delay we inserted. The actual D_hat is 4 less
                     // than this measured value

    ChParams ch_params;
    ch_params.D_hat = D_hat;
    ch_params.h_hat = h_hat;
    return ch_params;
}

/**
 * @brief Estimates strength of sinusoid driven by interferer for calibration
 *
 * This function estimates rss of inteferer.
 *
 * @param tx_usrp A pointer to the UHD USRP object for transmission.
 * @param NCapSamps Number of samples to be captured.
 * @param var Variance of the captured samples.
 * @return h_hat mag
 */
double IntfChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int& NCapSamps,
    const std::string& file)
{
    // Configure runtime
    // mode-------------------------------------------------------------------------------------------
    CompensateDelays(tx_usrp, 0);
    const std::uint32_t rx_ch_sel_bits      = 0b01;
    const std::uint32_t tx_core_bits        = 0b00;
    const std::uint32_t gpio_start_sel_bits = 0b00;
    mmio::WrMmio(tx_usrp, mmio::kDestChipCapEn, 0x0); // capture samples
    std::uint32_t mode_bits{0x0}; // sync mode

    mmio::StartTx(tx_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits, 0x0, 0x0);

    // Make sure dest is done recording before reading
    while (true) {
        mmio::ClearAddrBuffer(tx_usrp);
        if ((mmio::RdMmio(tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)== mmio::kCapMaxNumSamps - 1) // Dest has finished recording
            break;
    }
    mmio::ClearAddrBuffer(tx_usrp);

    // Read data
    std::vector<std::complex<double>> cap_samps = mmio::ReadSampleMem(tx_usrp, 0b1, NCapSamps, file);
    int N_w = static_cast<int>(cap_samps.size()); // number of captured samples

    std::vector<double> rx_if_envelope(N_w); // downconverted rx
    const double pi = std::acos(-1);
    for (int n = 0; n < N_w; ++n) {
        rx_if_envelope[n] = std::abs(cap_samps[n] * std::exp(std::complex<double>(0, -2 * pi / estim::kFwOsr * n))); 
    }

    double h_hat_mag = std::accumulate(rx_if_envelope.begin(), rx_if_envelope.end(), double(0)) / static_cast<double>(N_w);
    std::cout << "h_hat mag: " << h_hat_mag << std::endl;
    double rss_dbm = CalcRssdbW(std::complex<double>(h_hat_mag,0)) + 30; // convert to dbm

    return rss_dbm;
}

void SetSrcThreshold(
    const uhd::usrp::multi_usrp::sptr tx_usrp, std::complex<double> h_hat)
{
    double src_threshold          = std::pow(estim::kFbOsr * std::abs(h_hat), 2) / 2;
    uint32_t src_threshold_uint32 = static_cast<uint32_t>(
        std::round(src_threshold * (std::pow(2, mmio::kSrcThresholdFrac))));
    mmio::WrMmio(tx_usrp, mmio::kSrcThreshold, src_threshold_uint32);

    // std::cout << "h_hat_src mag: " << h_hat_mag << std::endl;
    // std::cout << "src threshold: " << src_threshold << std::endl;
}

/**
 * @brief After measuring the channel, initialize the destination
 *
 *
 * @param src_tx_usrp A pointer to the UHD USRP object for transmission.
 * @param D_test The test delay to be compensated.
 * @param NCapSamps Number of samples to be captured.
 * @param is_forward Whether to perform forward or feedback channel estimation.
 * @return ChParams struct containing d_hat and h_hat
 */
void ConfigDestIntfMitigation(const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    std::complex<double> h_hat,
    double chip_var)
{
    double pf = 0.001;
    boost::math::chi_squared chi2_dist(2 * estim::kDestMovingSumM);
    // Compute the quantile (inverse CDF)
    double chi_2_inv_val = boost::math::quantile(chi2_dist, 1 - pf);
    // std::cout << "chi_2_inv_val: " << chi_2_inv_val << std::endl;
    double var_threshold          = chip_var * chi_2_inv_val; // std::norm(h_hat)
    uint32_t var_threshold_uint32 = static_cast<uint32_t>(
        std::round(var_threshold * (std::pow(2, mmio::kDestVarThresholdFrac))));
    mmio::WrMmio(
        dest_tx_usrp, mmio::kDestVarThresholdAddr, var_threshold_uint32); // 0x7FFFFFFF

    double llr_threshold =
        estim::kDestLlrThreshold
        / (2 * estim::kDestMovingSumM
            * chip_var); //*std::sqrt(std::norm(h_hat)); //*std::norm(h_hat);
    uint32_t llr_threshold_uint32 = static_cast<uint32_t>(
        std::round(llr_threshold * (std::pow(2, mmio::kDestLlrThresholdFrac))));
        std::cout << std::hex << "llr_threshold: " << llr_threshold_uint32 << std::dec << std::endl;
    mmio::WrMmio(dest_tx_usrp, mmio::kDestThresholdAddr, llr_threshold_uint32); // 0x1

    double dest_if_chip_sig_energy_neg =
        -estim::kFwOsr * estim::kFwOsr * std::norm(h_hat);
    uint32_t dest_if_chip_sig_energy_neg_uint32 = static_cast<uint32_t>(std::round(
        dest_if_chip_sig_energy_neg * (std::pow(2, mmio::kDestIfChipSigEnergyNegFrac))));
    mmio::WrMmio(dest_tx_usrp,
        mmio::kDestIfChipSigEnergyNegAddr,
        dest_if_chip_sig_energy_neg_uint32); // 0x0

    double chip_var_inv          = 1 / (2 * estim::kDestMovingSumM * chip_var);
    uint32_t chip_var_inv_uint32 = static_cast<uint32_t>(
        std::round(chip_var_inv * (std::pow(2, mmio::kDestVarChipInvFrac))));
    mmio::WrMmio(dest_tx_usrp, mmio::kDestChipVarInvAddr, chip_var_inv_uint32);

    std::cout << "h_hat: " << h_hat << std::endl;
    std::cout << "chip_var: " << chip_var << std::endl;
    std::cout << "dest_if_chip_sig_energy_neg: " << dest_if_chip_sig_energy_neg
              << std::endl;
    std::cout << "var_threshold: " << var_threshold << std::endl;
    std::cout << "llr_threshold: " << llr_threshold << std::endl;
    std::cout << "chip_var_inv: " << chip_var_inv << std::endl;

    // std::cout << "h_hat_src mag: " << h_hat_mag << std::endl;
    // std::cout << "src threshold: " << src_threshold << std::endl;
}


// PrmblSweep
//  std::vector<int> D_hat_sweep, D_test_sweep;
//  std::vector<std::complex<double>> h_hat_sweep;
//  std::vector<double> SNR_sweep;

// int N_sweep_intervals = 0; //5
// const int DelaySweepInterval = 512;

// for(int interval_idx = -N_sweep_intervals/2; interval_idx <= N_sweep_intervals/2;
// interval_idx++) {
//     // Record the start time
//     auto start_time = std::chrono::high_resolution_clock::now();

//     int D_test = interval_idx * DelaySweepInterval; //D_test is the delay between src
//     and dest we set. This is the opposite of D_comp's logic

//     auto ch_params = estim::ChEstim(tx_usrp, D_test, rx_ch_sel_bits, tx_core_bits,
//     gpio_start_sel_bits, pow(2,12), ""); int D_hat = ch_params.D_hat;
//     std::complex<double> h_hat = ch_params.h_hat;

//     double SNR = estim::CalcSNR(h_hat, var);

//     D_test_sweep.push_back(D_test);
//     D_hat_sweep.push_back(D_hat);
//     h_hat_sweep.push_back(h_hat);
//     SNR_sweep.push_back(SNR);


//     //std::cout << "r[max_idx] = " << r[max_idx] << ", ";

//     std::cout << "D_test_sweep = " << D_test << ", ";
//     std::cout << "D_hat_sweep" << " = " << D_hat << ", ";
//     std::cout << "SNR_sweep" << " = " << SNR << ", ";
//     std::cout << "h_hat_sweep" << " : abs= " << std::abs(h_hat) << " arg= " <<
//     std::arg(h_hat) << std::endl;

//     auto end_time = std::chrono::high_resolution_clock::now();

//     // Calculate the elapsed time
//     auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time -
//     start_time);

//     // Print the result and execution time
//     std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
// }

// // Find the index of the maximum absolute value in vector r
// auto max_it = std::max_element(h_hat_sweep.begin(), h_hat_sweep.end(), [](const
// std::complex<double>& a, const std::complex<double>& b) {
//     return std::abs(a) < std::abs(b);
// }); //Finds the iterator pointing to the max element
// int max_idx = std::distance(h_hat_sweep.begin(), max_it); //Finds the index
// corresponding to that iterator

// std::complex<double> h_hat = h_hat_sweep[max_idx];
// int D_hat = D_hat_sweep[max_idx];

/**
 * Estimates the noise variance.
 *
 * This function sets the transmission amplitude to zero temporarily to measure noise,
 * captures samples from the USRP device, and then estimates the noise variance from these
 * samples. The problem with this approach is that this noise variance cant be multiplied
 * to estimate the chip noise, since the flicker noise is not iid white.
 *
 * @param tx_usrp The USRP device pointer from which to capture samples.
 * @param NCapSamps The number of samples to capture for noise estimation.
 * @param rx_ch_sel_bits The bits to select the RX channel.
 * @param file The name of the file to save the captured samples.
 * @return The estimated noise variance.
 */
double EstimNoise(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int NCapSamps,
    const uint32_t rx_ch_sel_bits,
    const std::string& file)
{
    // temporarily set tx_amp = 0
    uint32_t tx_amp = mmio::RdMmio(tx_usrp, mmio::kSrcTxAmpAddr);
    mmio::WrMmio(tx_usrp, mmio::kSrcTxAmpAddr, 0x0);

    mmio::WrMmio(tx_usrp, mmio::kDestChipCapEn, 0x0); // capture chips for sample analysis

    mmio::StartTx(tx_usrp,
        0x0,
        rx_ch_sel_bits,
        0x0,
        0x0,
        0x0,
        0x0); // Mode zero, only listen at src (no tx)

    // Typically, capture is so fast no delay is needed
    while (true) {
        // Run and check received pkt
        mmio::WrMmio(tx_usrp, 0x0, 0x0);
        if ((mmio::RdMmio(tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)
            == mmio::kCapMaxNumSamps - 1) // Dest has finished recording
            break;
    }

    mmio::WrMmio(
        tx_usrp, 0x0, 0x0); // clear buffer since readsamplemem looks at kDestCapIdx

    // Read on chip acquired data and write to binary file to be parsed by matlab
    std::vector<std::complex<double>> cap_samps =
        mmio::ReadSampleMem(tx_usrp, 0b1, NCapSamps, file);

    // Estimate noise
    std::complex<double> sum = std::accumulate(
        std::begin(cap_samps), std::end(cap_samps), std::complex<double>{0, 0});
    // long unsigned int size->double is a narrowing but hopefully our vectors dont have
    // this size
    std::complex<double> mu =
        sum / std::complex<double>{static_cast<double>(cap_samps.size()), 0};

    double accum = 0;
    std::for_each(std::begin(cap_samps),
        std::end(cap_samps),
        [&](const std::complex<double> d) { accum += std::pow(std::abs(d - mu), 2); });

    double var = accum / (2 * cap_samps.size() - 1); // 2* I and Q are 2 separate samples

    mmio::WrMmio(tx_usrp, mmio::kSrcTxAmpAddr, tx_amp);
    return var;
}

/**
 * Estimates the noise variance.
 *
 * This function sets the transmission amplitude to zero temporarily to measure noise,
 * captures chips from the USRP device, and then estimates the chip noise variance from
 * these samples.
 *
 * This is done because it allows us to capture baseband flicker noise
 *
 * @param tx_usrp The USRP device pointer from which to capture samples.
 * @param NCapSamps The number of samples to capture for noise estimation.
 * @param rx_ch_sel_bits The bits to select the RX channel.
 * @param file The name of the file to save the captured samples.
 * @return The estimated noise variance.
 */
double P2PEstimChipNoise(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int NCapSamps,
    const std::string& file)
{
    // temporarily set tx_amp = 0
    uint32_t tx_amp = mmio::RdMmio(src_tx_usrp, mmio::kSrcTxAmpAddr);
    mmio::WrMmio(src_tx_usrp, mmio::kSrcTxAmpAddr, 0x0);

    // set phase equalization to 1
    mmio::WrMmio(dest_tx_usrp, mmio::kDestChEqReAddr, 0x00002000);
    mmio::WrMmio(dest_tx_usrp, mmio::kDestChEqImAddr, 0x00000000);

    mmio::WrMmio(
        dest_tx_usrp, mmio::kDestChipCapEn, 0x1); // capture chips for sample analysis

    std::uint32_t mode_bits = 0b00; // connect afe to src module
    mmio::P2PStartTxRx(src_tx_usrp,
        dest_tx_usrp,
        mode_bits,
        estim::kFwdGpioStartSelBits,
        0x0,
        0x0,
        0x0,
        false);

    // mmio::ReadBBCore(dest_tx_usrp);

    // Typically, capture is so fast no delay is needed
    while (true) {
        // Run and check received pkt
        mmio::ClearAddrBuffer(dest_tx_usrp);
        if ((mmio::RdMmio(dest_tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)
            >= mmio::kCapMaxNumSamps
                   - 2) // around 100 ms...not sure why it still doesnt hit FFFF
            break;
    }

    // Read on chip acquired data and write to binary file to be parsed by matlab
    mmio::ClearAddrBuffer(dest_tx_usrp);
    std::vector<double> cap_samps = mmio::ReadChipMem(dest_tx_usrp, 0b1, NCapSamps, file);

    // Estimate noise
    double sum = std::accumulate(std::begin(cap_samps), std::end(cap_samps), 0.0);
    // long unsigned int size->double is a narrowing but hopefully our vectors dont have
    // this size
    double mu = sum / static_cast<double>(cap_samps.size());
    std::cout << "mu = " << mu << std::endl;
    double accum = 0;
    std::for_each(std::begin(cap_samps), std::end(cap_samps), [&](const double d) {
        accum += std::pow(d - mu, 2);
    });

    double var = accum / (cap_samps.size() - 1);

    mmio::WrMmio(
        src_tx_usrp, mmio::kSrcTxAmpAddr, tx_amp); // restore previous gain setting

    return var;
}

/**
 * Estimates the noise variance.
 *
 * This function sets the transmission amplitude to zero temporarily to measure noise,
 * captures chips from the USRP device, and then estimates the noise variance from these
 * samples.
 *
 * This is done because it allows us to capture baseband flicker noise
 *
 * @param tx_usrp The USRP device pointer from which to capture samples.
 * @param NCapSamps The number of samples to capture for noise estimation.
 * @param rx_ch_sel_bits The bits to select the RX channel.
 * @param file The name of the file to save the captured samples.
 * @return The estimated noise variance.
 */
double EstimChipNoise(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int NCapSamps,
    const uint32_t rx_ch_sel_bits,
    const std::string& file)
{
    // temporarily set tx_amp = 0
    uint32_t tx_amp = mmio::RdMmio(tx_usrp, mmio::kSrcTxAmpAddr);
    mmio::WrMmio(tx_usrp, mmio::kSrcTxAmpAddr, 0x0);
    mmio::WrMmio(tx_usrp, mmio::kDestChipCapEn, 0x1); // capture chips for sample analysis

    mmio::StartTx(tx_usrp,
        0b00,
        rx_ch_sel_bits,
        0x0,
        0x0,
        0x0,
        0x0); // Use the active pkt fctn, but capture noise instead of sufficient
              // statistic

    // Typically, capture is so fast no delay is needed
    while (true) {
        // Run and check received pkt
        mmio::ClearAddrBuffer(tx_usrp);
        if ((mmio::RdMmio(tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask)
            >= mmio::kCapMaxNumSamps - 2) // Dest has finished recording
            break;
    }

    // mmio::ReadBBCore(tx_usrp);

    // Read on chip acquired data and write to binary file to be parsed by matlab
    std::vector<double> cap_samps = mmio::ReadChipMem(tx_usrp, 0b1, NCapSamps, file);

    // Estimate noise
    double sum = std::accumulate(std::begin(cap_samps), std::end(cap_samps), 0.0);
    // long unsigned int size->double is a narrowing but hopefully our vectors dont have
    // this size
    double mu = sum / static_cast<double>(cap_samps.size());
    std::cout << "mu = " << mu << std::endl;
    double accum = 0;
    std::for_each(std::begin(cap_samps), std::end(cap_samps), [&](const double d) {
        accum += std::pow(d - mu, 2);
    });

    double var = accum / (cap_samps.size() - 1);

    mmio::WrMmio(tx_usrp, mmio::kSrcTxAmpAddr, tx_amp);
    return var;
}

// /**
//  * @brief Estimates and prints noise parameters based on chip variance.
//  *
//  * This function computes and logs three values based on the input chip variance (`var`):
//  *  - Estimated chip variance
//  *  - Received noise power in dBm (`rx_noise_dbW`)
//  *  - Estimated noise spectral density `N0` in dB (assumes 5 MHz bandwidth and 336 chips)
//  *
//  * The calculations take into account:
//  *  - ADC gain of 41.81 dB from antenna to digital domain
//  *  - ADC full-scale swing of 2V with 14-bit resolution
//  *  - A 50-ohm load for power normalization
//  *  - Oversampling ratio defined by `estim::kFwOsr`
//  *
//  * @param var The measured chip power variance (should reflect the power of received noise
//  * samples).
//  */
// double CalcNoiseRssDbw(double chip_var)
// {
//     //THIS FUNCTION IS NOT CORRECT. IT ASSUMES THE NOISE VARIANCE IS AT THE ANTENNA, NOT THE INTERNALS OF THE RECEIVER. DO NOT USE
//     // Gain from antenna to ADC is 41.81 dB, ADC swing is 2V, 14-bit resolution
//     double noise_rss_dbw =
//         10 * std::log10(chip_var)
//         + 20 * std::log10(1.0 / estim::kFwOsr) // find the power in this chip
//         + 20 * std::log10(std::pow(2, -13)) - estim::rx_gain
//         - 10 * std::log10(50); // 50-ohm termination

//     std::cout << "rx_noise (dbW)= " << noise_rss_dbw << std::endl;

//     // Estimated N0 for a 5 MHz bandwidth and 336 chips. The noise equaivalent bandwidth (i.e. the power of the filter is 1 / (5.0e-9 * 336.0))
//     double estimated_N0 = -10 * std::log10(1 / (2.0* 5.0e-9 * 336.0)) + noise_rss_dbw + 30;
//     std::cout << "Estimated N0 (dbm)= " << estimated_N0 << std::endl;

//     return noise_rss_dbw;
// }

/**
 * @brief Estimate the noise spectral density \( N_0 \) in dBm.
 *
 * This function computes the estimated noise power spectral density \( N_0 \) 
 * in dBm based on the received signal strength (RSS) in dBm and the 
 * energy-per-symbol to noise ratio (Es/N0) in dB.
 *
 * The underlying assumption is that the signal occupies a bandwidth determined 
 * by 336 chips over 32 symbols, each with a chip period of 5 ns.
 * The formula used is:
 * \[
 * N_0\text{(dBm)} = \text{RSS(dBm)} + 10 \log_{10}(32 \cdot 336 \cdot 5 \cdot 10^{-9}) - \text{Es/N0(dB)}
 * \]
 *
 * @param EsN0_db  The Es/N0 value in dB.
 * @param rss_dbm  The received signal strength in dBm.
 * @return The estimated \( N_0 \) in dBm.
 */
double CalcN0dbm(double EsN0_db, double rss_dbm)
{
    double N0_dbm = rss_dbm + 10 * std::log10((32.0 * 336.0 * 5.0e-9)) - EsN0_db;
    std::cout << "Estimated N0 (dbm)= " << N0_dbm << std::endl;
    return N0_dbm;
}

/**
 * @brief Calculates and prints the received signal strength (RSS) in dBW.
 *
 * This function estimates the received signal strength based on the provided forward
 * channel estimate (`h_hat_fwd`). It assumes:
 *  - A 14-bit ADC with a full-scale voltage swing of 2V
 *  - A total gain of 41.81 dB from antenna to ADC
 *  - A 50-ohm termination for power calculation
 *
 * The RSS is calculated as:
 * \f[
 * \text{RSS}_{\text{dBW}} = 20 \log_{10}(|h\_hat| \cdot 2^{-13}) - 41.81 - 10
 * \log_{10}(50)
 * \f]
 * and then printed to the standard output in dBm.
 *
 * @param h_hat The estimated forward channel coefficient (linear scale, not dB).
 */
double CalcRssdbW(std::complex<double> h_hat)
{
    // rx_gain used to be 41.81 not sure why this changed
    double rss_dbW = 20 * log10(std::abs(h_hat) * std::pow(2, -13)) - estim::rx_gain
                     - 10 * log10(50); // 50 ohm resistor at end
    // std::cout << "Estimated rss (dbW)= " << rss_dbW << std::endl;
    return rss_dbW;
}


double CalcSNR(const std::complex<double>& h_hat, const double var)
{
    double SNR = 10 * std::log10(std::pow(prmbl_amp * std::abs(h_hat), 2) / (var * 2));
    return SNR;
}

double CalcEsN0(const std::complex<double>& h_hat, const int osr, const double var)
{
    double EsN0 =
        10 * std::log10(std::pow(prmbl_amp * std::abs(h_hat), 2) * osr / (var * 2));
    return EsN0;
}

/**
 * Calculate the EsN0 using the measured chip variance and the signal amplitude. This is
 * actually the SYMBOL EsN0!!!! This doesnt work for different thresholds which change the
 * rate!!!!!
 */
double CalcChipEsN0(const std::complex<double>& h_hat, const double chip_var)
{
    double EsN0 = 10 * std::log10(estim::kNChips * std::pow(estim::kFwOsr * std::abs(h_hat), 2) / (chip_var * 2));
    return EsN0;
}

/**
 * Assuming the analog gain is set so that the received amplitude is higher than the
 * expected operating point of the receiver, this code decreases tx_gain so that the
 * maximum snr possible is achieved (fpga code cant shift right). This is enough for
 * analog experiments
 *
 * You will probably need to set the tx-gain quite high, ex 15 dB
 */
void MaxSnrConfig(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const std::complex<double> h_hat,
    const double measured_EsN0)
{
    std::complex<double> h_comp =
        h_hat / std::abs(h_hat); // Set digital rx gain to be one, and scale the tx down
                                 // to meet the target gain
    estim::PhaseEq(tx_usrp, h_comp);

    double h_mag      = std::abs(h_hat);
    double total_gain = -20 * std::log10(h_mag); // total gain needed in system to
                                                 // equalize
    std::cout << "total gain: " << total_gain << std::endl;

    double digital_gain     = total_gain;
    double lin_digital_gain = std::pow(10, digital_gain / 20);
    uint16_t tx_amp =
        static_cast<uint16_t>(std::round(lin_digital_gain * std::pow(2, 16) - 1));
    std::cout << "tx_amp = " << tx_amp << std::endl;
    mmio::WrMmio(tx_usrp, mmio::kSrcTxAmpAddr, tx_amp);

    if (lin_digital_gain > 1) {
        std::cerr << "MaxSnrConfig >> Error: digital_tx_gain = " << lin_digital_gain
                  << "> 1, increase tx_gain and try again." << std::endl;
        return;
    }

    std::cout << "Running test with EsN0: " << measured_EsN0 + total_gain << std::endl;
}


/**
 * Delays destination. To purposefully insert a delay of D, set D_hat = D
 *
 * @param D_hat estimated relative delay of source wrt destination
 *
 */
void CompensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat)
{
    uint16_t dest_delay, src_delay;
    if (D_hat < 0) { // D < 0 -> prmbl early -> delay src
        dest_delay = 0;
        src_delay  = -D_hat;
    } else { // D > 0 -> prmbl late -> delay dest
        dest_delay = D_hat;
        src_delay  = 0;
    }
    mmio::wr_mem_cmd(tx_usrp, (mmio::kSrcDelayAddr << 32) | src_delay);
    mmio::wr_mem_cmd(tx_usrp, (mmio::kDestDelayAddr << 32) | dest_delay);
}

/**
 * Delays destination. To purposefully insert a delay of D, set D_hat = D
 *
 * @param D_hat estimated relative delay of source wrt destination
 *
 */
void P2PCompensateDelays(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int D_hat)
{
    uint16_t dest_delay, src_delay;
    if (D_hat < 0) { // D < 0 -> prmbl early -> delay src
        dest_delay = 0;
        src_delay  = -D_hat;
    } else { // D > 0 -> prmbl late -> delay dest
        dest_delay = D_hat;
        src_delay  = 0;
    }
    mmio::wr_mem_cmd(src_tx_usrp, (mmio::kSrcDelayAddr << 32) | src_delay);
    mmio::wr_mem_cmd(dest_tx_usrp, (mmio::kDestDelayAddr << 32) | dest_delay);
}


const int kEqFrac = 13;

/**
 * Sets the digital flatfading equalization parameters for the destination
 *
 * h_hat magnitude can range from 0.5 to 2. This should be applied after analog gain/tx
 * amp is set. Sometimes, writing the phase may fail. Its unclear why this happens
 *
 * @param h_hat The channel estimate in complex double format.
 * @param tx_usrp A shared pointer to the UHD multi_usrp object for transmitting.
 * @return int containing success of function
 */
int PhaseEq(uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double>& h_hat)
{
    std::complex<double> reciprocal_h_hat = 1.0 / h_hat;

    if (std::abs(h_hat) < .5 || std::abs(h_hat) > 1.5) {
        std::cerr << "Error: Digital compensation of h_mag is out of rand [0.66,2]. "
                     "rx-gain is not set correctly or signal power is too high"
                  << std::endl;
        reciprocal_h_hat = std::abs(h_hat) / h_hat;
    }

    // std::cout << reciprocal_h_hat << std::endl;
    double dest_ch_eq_re = std::real(reciprocal_h_hat) * std::pow(2, kEqFrac);
    double dest_ch_eq_im =
        -std::imag(reciprocal_h_hat)
        * std::pow(2,
            kEqFrac); // because dest expects the phase of the channel, not the reciprocal

    // covert to bit command
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

    mmio::ClearAddrBuffer(tx_usrp);
    mmio::WrMmio(
        tx_usrp, mmio::kDestChEqReAddr, static_cast<uint16_t>(dest_ch_eq_re_int16));
    mmio::WrMmio(
        tx_usrp, mmio::kDestChEqImAddr, static_cast<uint16_t>(dest_ch_eq_im_int16));

    auto dest_ch_eq_re_int16_mem = mmio::RdMmio(tx_usrp, mmio::kDestChEqReAddr, false);
    if (static_cast<uint16_t>(dest_ch_eq_re_int16) != dest_ch_eq_re_int16_mem) {
        std::cout << std::hex
                  << "Attempted to set dest_ch_eq_re_int16 = " << dest_ch_eq_re_int16
                  << std::endl;
        std::cout << std::hex << "Actual value: " << dest_ch_eq_re_int16_mem << std::endl;
        return 1;
    }

    auto dest_ch_eq_im_int16_mem = mmio::RdMmio(tx_usrp, mmio::kDestChEqImAddr, false);
    if (static_cast<uint16_t>(dest_ch_eq_im_int16) != dest_ch_eq_im_int16_mem) {
        std::cout << std::hex
                  << "Attempted to set dest_ch_eq_im_int16 = " << dest_ch_eq_im_int16
                  << std::endl;
        std::cout << std::hex << "Actual value: " << dest_ch_eq_im_int16_mem << std::endl;
        return 1;
    }

    return 0;
}

/**
 * @brief Computes the cross-correlation of two complex-valued vectors.
 *
 * The function calculates the cross-correlation between two input vectors, `x` and `y`,
 * and stores the results in the output vectors `r` and `lags`. Cross-correlation is a
 * measure of similarity between two signals as a function of the time lag applied to one
 * of them.
 *
 * @param x Input vector representing the first complex signal. It is conjugated and
 * dragged over y
 * @param y Input vector representing the second complex signal.
 * @param r Output vector containing the cross-correlation values.
 * @param lags Output vector containing the corresponding time lags for each
 * cross-correlation value.
 *
 * @details The function uses a nested loop to iterate over all possible time lags within
 * the range [-n+1, m-1]. For each lag, it calculates the cross-correlation sum by
 * multiplying the complex conjugate of `x` with the corresponding element in `y`. The
 * results are stored in the output vectors `r` and `lags`.
 *
 * @note The input vectors `x` and `y` must be of complex type, and the output vectors `r`
 * and `lags` are modified in place.
 *
 */
void XcorrSlow(const std::vector<std::complex<double>>& x,
    const std::vector<std::complex<double>>& y,
    std::vector<std::complex<double>>& r,
    std::vector<int>& lags)
{
    int n = static_cast<int>(x.size());
    int m = static_cast<int>(y.size());

    int maxLag = n + m - 1; // Maximum lag for cross-correlation

    r.resize(
        maxLag, std::complex<double>(0.0, 0.0)); // Initialize result vector with zeros
    lags.resize(maxLag, 0); // Initialize lags vector

    for (int lag = -n + 1; lag < m; ++lag) {
        std::complex<double> sum(0.0, 0.0);
        int startIdxX = std::max(0, -lag);
        int endIdxX   = std::min(n - 1, m - 1 - lag);

        for (int i = startIdxX; i <= endIdxX; ++i) {
            int j = i + lag;
            sum += std::conj(x[i]) * y[j];
        }

        int rIdx   = lag + n - 1;
        r[rIdx]    = sum;
        lags[rIdx] = lag;
    }
}

// Template function to generate MATLAB array creation code with custom vector name
template <typename T>
std::string generateMatlabArray(
    const std::vector<T>& array, const std::string& vectorName)
{
    static_assert(std::is_arithmetic<T>::value, "Template argument must be numeric");

    std::ostringstream matlabCode;
    matlabCode << vectorName << " = [";

    for (size_t i = 0; i < array.size(); ++i) {
        matlabCode << array[i];
        if (i < array.size() - 1) {
            matlabCode << ", ";
        }
    }

    matlabCode << "];\n";

    return matlabCode.str();
}
// Explicit instantiation for types you plan to use
template std::string generateMatlabArray(
    const std::vector<double>& array, const std::string& vectorName);
template std::string generateMatlabArray(
    const std::vector<int>& array, const std::string& vectorName);

/**
 * @brief Upsamples a vector by duplicating each sample N times.
 *
 * This template function takes a vector of either arithmetic types or
 * std::complex<double> and an upsampling factor 'N'. It creates and returns a new vector
 * where each element from the original vector is duplicated 'N' times.
 *
 * @tparam T The type of elements in the vector (arithmetic type or std::complex<double>).
 * @param input The original vector to be upsampled.
 * @param N The upsampling factor. Each sample in the original vector will be duplicated
 * 'N' times.
 * @return A new vector representing the upsampled data.
 */
template <typename T>
std::vector<T> Upsample(const std::vector<T>& input, int N)
{
    static_assert(
        std::is_arithmetic<T>::value || std::is_same<T, std::complex<double>>::value,
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
template std::vector<double> Upsample(const std::vector<double>& input, int N);
template std::vector<std::complex<double>> Upsample(
    const std::vector<std::complex<double>>& input, int N);
} // namespace estim
