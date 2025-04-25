#ifndef ESTIM_H
#define ESTIM_H
// The only time you should include a header within another .h file is if you need to
// access a type definition in that header
#include <uhd/usrp/multi_usrp.hpp>
#include <cstdint>

namespace estim {
// Device params used in estimating SNR
constexpr int kFwOsr        = 336;
constexpr int kFbOsr        = 16;
constexpr int kNChips       = 32; // average number of chips per symbol
constexpr int kSrcProcDelay = 4; // samples it takes to process data at source

constexpr int kDestMovingSumM = 96;
constexpr double kDestLlrThreshold = 9600; // unscaled llr threshold value. This will be scaled based on implementation.
//10051 leads to 32.1 sym length
// measurements---
constexpr double rx_gain = 41.81; // gain of the receiver. used to be 41.81??? 18.237

// P2P communication contains a GPIO channel from source to destination for signalling
// when the source starts
constexpr std::uint32_t kFwdGpioStartSelBits = 0b01; // dest listens to gpio for start
// For feedback channel estimation, have the "dest" usrp transmit preamble to the source.
// For start signalling, have the source usrp send the start and then begin listening. in
// this case, the gpio start bits are reversed, since the destination uses the source
// module and now the source module must listen to gpio
constexpr std::uint32_t kFbGpioStartSelBits = 0b10; // dest listens to gpio for start

constexpr double kMaxTxGain{31.5}, kMaxRxGain{31.5};
constexpr double kMinTxGain{0}, kMinRxGain{0};

constexpr double kDBPerBit = 20 * std::log10(2);
// void startGNUSocket(bool tx_on, double intf_rss_dbm, double target_intf_rss_dbm);

constexpr int kServerPort       = 12345;

// Create a struct for the header message
struct MSG_t
{
    bool event;
    double intf_rss_dbm;
    double target_intf_rss_dbm;
};

bool send_message(int sock, bool event, double intf_rss_dbm, double target_intf_rss_dbm);
int connectToServerSock();

void XcorrSlow(const std::vector<std::complex<double>>& x,
    const std::vector<std::complex<double>>& y,
    std::vector<std::complex<double>>& r,
    std::vector<int>& lags);
struct ChParams
{
    int D_hat; // delay of the channel
    std::complex<double> h_hat; // fading coefficient of the channel

    // Default constructor
    ChParams()
        : D_hat(0), h_hat(0.0, 0.0) {} // Initializes D_hat to 0 and h_hat to (0, 0)
};
ChParams ChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int D_test,
    const std::uint32_t rx_ch_sel_bits,
    const std::uint32_t tx_core_bits,
    const std::uint32_t gpio_start_sel_bits,
    const int& NCapSamps,
    const std::string& file);
ChParams FbChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int D_test,
    const std::uint32_t rx_ch_sel_bits,
    const std::uint32_t tx_core_bits,
    const std::uint32_t gpio_start_sel_bits,
    const int& NCapSamps,
    const std::string& file);
double IntfChEstim(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int& NCapSamps,
    const std::string& file);

double EstimNoise(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int NCapSamps,
    const uint32_t rx_ch_sel_bits = 0b01,
    const std::string& file       = "");
double EstimChipNoise(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const int NCapSamps,
    const uint32_t rx_ch_sel_bits,
    const std::string& file = "");
double CalcNoiseRssDbm(double chip_var);
double CalcRssdbW(std::complex<double> h_hat);
double CalcSNR(const std::complex<double>& h_hat, const double var);
double CalcEsN0(const std::complex<double>& h_hat, const int osr, const double var);
double CalcChipEsN0(const std::complex<double>& h_hat, const double chip_var);

void MaxSnrConfig(const uhd::usrp::multi_usrp::sptr tx_usrp,
    const std::complex<double> h_hat,
    const double measured_EsN0);
void ConfigDestIntfMitigation(const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    std::complex<double> h_hat,
    double chip_var);


void CompensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat);
int PhaseEq(uhd::usrp::multi_usrp::sptr tx_usrp, const std::complex<double>& h_hat);

void SetSrcThreshold(
    const uhd::usrp::multi_usrp::sptr tx_usrp, std::complex<double> h_hat);

double P2PEstimChipNoise(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int NCapSamps,
    const std::string& file);
ChParams P2PChEstim(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int D_test,
    const int& NCapSamps,
    const bool is_forward,
    const uint32_t start_sync,
    const bool skip_rst,
    const std::string& file);
void P2PCompensateDelays(const uhd::usrp::multi_usrp::sptr src_tx_usrp,
    const uhd::usrp::multi_usrp::sptr dest_tx_usrp,
    const int D_hat);

template <typename T>
std::string generateMatlabArray(
    const std::vector<T>& array, const std::string& vectorName);

template <typename T>
std::vector<T> Upsample(const std::vector<T>& input, int N);
} // namespace estim

#endif // ESTIM_H
