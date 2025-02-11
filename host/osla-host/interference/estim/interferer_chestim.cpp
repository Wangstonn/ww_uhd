#include "estim.h"
#include "../mmio/mmio.h"
#include <vector>
#include <complex>
#include <numeric>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>
#include <sstream>
#include <type_traits>

namespace estim {

    /**
     * @brief Estimates channel characteristics from external interferer
     * 
     * This function estimates channel coefficient for an external interferer.
     * Please verify Interferer is configured correctly to pilot file used.
     * 
     * @param src_tx_usrp A pointer to the UHD USRP object for transmission.
     * @param dest_tx_usrp A pointer to the UHD USRP object for receive.
     * @param NCapSamps Number of samples to be captured.
     * @param file filename to write data for post-processing
     */
    void record_interference(const uhd::usrp::multi_usrp::sptr src_tx_usrp,,const uhd::usrp::multi_usrp::sptr dest_tx_usrp, const int& NCapSamps, const std::string& file) {
        
        //temporarily set tx_amp = 0
        uint32_t tx_amp = mmio::RdMmio(src_tx_usrp, mmio::kSrcTxAmpAddr);
        mmio::WrMmio(src_tx_usrp,mmio::kSrcTxAmpAddr,0x0);

        mmio::WrMmio(dest_tx_usrp, mmio::kDestChipCapEn, 0x0); //capture samples

        std::uint32_t mode_bits = 0b01; //connect afe to src module
        mmio::P2PStartTxRx(src_tx_usrp, dest_tx_usrp, mode_bits, estim::kFwdGpioStartSelBits,0x0,0x0, false);

        //Typically, capture is so fast no delay is needed
        while(true) {
            //Run and check received pkt   
            mmio::ClearAddrBuffer(dest_tx_usrp);
            if((mmio::RdMmio(dest_tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask) >= mmio::kCapMaxNumSamps-2) //around 100 ms...not sure why it still doesnt hit FFFF
                break;
        }

        //Read on chip acquired data and write to binary file to be parsed by matlab
        mmio::ClearAddrBuffer(dest_tx_usrp);
        std::vector<std::complex<double>> cap_samps = mmio::ReadSampleMem(dest_tx_usrp, 0b1, NCapSamps, file);

        mmio::WrMmio(src_tx_usrp,mmio::kSrcTxAmpAddr,tx_amp); //restore previous gain setting
    }



        /**
     * @brief Estimates channel characteristics from external interferer
     * 
     * This function estimates channel coefficient for an external interferer.
     * Please verify Interferer is configured correctly to pilot file used.
     * 
     * @param src_tx_usrp A pointer to the UHD USRP object for transmission.
     * @param dest_tx_usrp A pointer to the UHD USRP object for receive.
     * @param NCapSamps Number of samples to be captured.
     * @param file filename to write data for post-processing
     */
    double estimInterference(const uhd::usrp::multi_usrp::sptr src_tx_usrp,,const uhd::usrp::multi_usrp::sptr dest_tx_usrp, const int& NCapSamps, const std::string& file) {
        
        //temporarily set tx_amp = 0
        uint32_t tx_amp = mmio::RdMmio(src_tx_usrp, mmio::kSrcTxAmpAddr);
        mmio::WrMmio(src_tx_usrp,mmio::kSrcTxAmpAddr,0x0);

        mmio::WrMmio(dest_tx_usrp, mmio::kDestChipCapEn, 0x0); //capture samples

        std::uint32_t mode_bits = 0b01; //connect afe to src module
        mmio::P2PStartTxRx(src_tx_usrp, dest_tx_usrp, mode_bits, estim::kFwdGpioStartSelBits,0x0,0x0, false);

        //Typically, capture is so fast no delay is needed
        while(true) {
            //Run and check received pkt   
            mmio::ClearAddrBuffer(dest_tx_usrp);
            if((mmio::RdMmio(dest_tx_usrp, mmio::kDestCapIdxAddr) & mmio::kCapIdxMask) >= mmio::kCapMaxNumSamps-2) //around 100 ms...not sure why it still doesnt hit FFFF
                break;
        }

        //Read on chip acquired data and write to binary file to be parsed by matlab
        mmio::ClearAddrBuffer(dest_tx_usrp);
        std::vector<std::complex<double>> cap_samps = mmio::ReadSampleMem(dest_tx_usrp, 0b1, NCapSamps,file);

        mmio::WrMmio(src_tx_usrp,mmio::kSrcTxAmpAddr,tx_amp); //restore previous gain setting

        //generate complex exponential
        int N_w = static_cast<int>(cap_samps.size()); //number of captured samples

        std::complex<double> Pr = std::complex<double>(0,0);
        const double pi = std::acos(-1);

        for (int n = 0; n < N_w; ++n) {
            Pr += cap_samps[n] * std::exp(std::complex<double>(0, -2*pi/estim::kFwOsr*n));
        }

        return 20*std::log10(abs(Pr/(N_w))) - 137;
    }
}