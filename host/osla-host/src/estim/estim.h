#ifndef ESTIM_H
#define ESTIM_H
//The only time you should include a header within another .h file is if you need to access a type definition in that header
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

void xcorr_slow(const std::vector<std::complex<double>>& x, const std::vector<std::complex<double>>& y, std::vector<std::complex<double>>& r, std::vector<int>& lags);
struct ChParams {
    int D_hat; //delay of the channel
    std::complex<double> h_hat; //fading coefficient of the channel

     // Default constructor
    ChParams() : D_hat(0), h_hat(0.0, 0.0) {} // Initializes D_hat to 0 and h_hat to (0, 0)
};
ChParams ch_estim(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_test, const std::uint32_t rx_ch_sel_bits, const std::uint32_t tx_core_bits, const std::uint32_t gpio_start_sel_bits, const int& NCapSamps, const std::string& file);
double EstimNoise(const uhd::usrp::multi_usrp::sptr tx_usrp, const int NCapSamps, const uint32_t rx_ch_sel_bits = 0b01);
double calcSNR(const std::complex<double>& h_hat, const double var);
double calcEsN0(const std::complex<double>& h_hat, const int osr, const double var);


void compensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat);
void PhaseEq(uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double>& h_hat);

template <typename T>
std::vector<T> upsample(const std::vector<T>& input, int N);


#endif  // ESTIM_H
