#ifndef ESTIM_H
#define ESTIM_H
//The only time you should include a header within another .h file is if you need to access a type definition in that header
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

namespace estim {
    //Device params used in estimating SNR
    constexpr int kFwOsr = 336;
    constexpr int kFbOsr = 16; 
    constexpr int kNChips = 32; //average number of chips per symbol
    constexpr int kSrcProcDelay = 4; //samples it takes to process data at source

    constexpr double kMaxTxGain{31.5}, kMaxRxGain{31.5};
    constexpr double kMinTxGain{0}, kMinRxGain{0};

    constexpr double kDBPerBit = 20*std::log10(2);
    void XcorrSlow(const std::vector<std::complex<double>>& x, const std::vector<std::complex<double>>& y, std::vector<std::complex<double>>& r, std::vector<int>& lags);
    struct ChParams {
        int D_hat; //delay of the channel
        std::complex<double> h_hat; //fading coefficient of the channel

        // Default constructor
        ChParams() : D_hat(0), h_hat(0.0, 0.0) {} // Initializes D_hat to 0 and h_hat to (0, 0)
    };
    ChParams ChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_test, const std::uint32_t rx_ch_sel_bits, const std::uint32_t tx_core_bits, const std::uint32_t gpio_start_sel_bits, const int& NCapSamps, const std::string& file);
    double EstimNoise(const uhd::usrp::multi_usrp::sptr tx_usrp, const int NCapSamps, const uint32_t rx_ch_sel_bits = 0b01, const std::string& file = "");
    double EstimChipNoise(const uhd::usrp::multi_usrp::sptr tx_usrp, const int NCapSamps, const uint32_t rx_ch_sel_bits, const std::string& file = "");
    double CalcSNR(const std::complex<double>& h_hat, const double var);
    double CalcEsN0(const std::complex<double>& h_hat, const int osr, const double var);
    double CalcChipEsN0(const std::complex<double>& h_hat, const double chip_var);

    void MaxSnrConfig(const uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double> h_hat, const double measured_EsN0);

    void CompensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat);
    int PhaseEq(uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double>& h_hat);

    template <typename T>
    std::vector<T> Upsample(const std::vector<T>& input, int N);
}

#endif  // ESTIM_H
