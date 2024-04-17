#ifndef MMIO_H
#define MMIO_H
//The only time you should include a header within another .h file is if you need to access a type definition in that header
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

namespace mmio {
    //Bitmasks to help read and "assemble" commands
    constexpr std::uint64_t cmdBits{0xF0000000'00000000}; 
    constexpr std::uint64_t addrBits{0x0FFFFFFF'00000000};
    constexpr std::uint64_t dataBits{0x00000000'FFFFFFFF};

    //Useful write commands
    constexpr uint64_t rst_cmd = 0x80000001'00000001;
    constexpr uint64_t src_delay_cmd = 0x80000011;
    constexpr uint64_t dest_delay_cmd = 0x80000033;


    //Compensation
    constexpr uint32_t kDestChEqReAddr = 0x31;
    constexpr uint32_t kDestChEqImAddr = 0x32;
    constexpr uint32_t kSrcTxAmpAddr = 0x34;

    constexpr int kPktLen = 256;
    constexpr uint32_t kInPktAddr = 0x20;
    constexpr uint32_t kOutPktAddr = 0x810;

    //Useful read commands
    //constexpr std::uint32_t isDone{0x08000001}; //src done tx
    constexpr std::uint32_t kBbStatusAddr{0x800}; 


    uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, bool verbose = false);
    void WrMmio(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t addr, const uint32_t data);
    void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint64_t cmd);
    void read_sample_mem(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const int NCapSamps, const std::string& file);
    //void read_samples_helper(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, std::ofstream& of_file);
    void start_tx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits);
    void InitBBCore (uhd::usrp::multi_usrp::sptr tx_usrp);
    void ReadBBCore (uhd::usrp::multi_usrp::sptr tx_usrp);
}

#endif  // MMIO_H
