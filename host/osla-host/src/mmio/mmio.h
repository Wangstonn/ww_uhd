#ifndef MMIO_H
#define MMIO_H
//The only time you should include a header within another .h file is if you need to access a type definition in that header
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

//Bitmasks to help read and "assemble" commands
constexpr std::uint32_t cmdBits{0xF0000000}; 
constexpr std::uint32_t addrBits{0x0FFF0000};
constexpr std::uint32_t dataBits{0x0000FFFF};

//Useful write commands
constexpr uint32_t rst_cmd = 0x80010001;
//Useful read commands
constexpr std::uint32_t isDone{0x08000001}; //src done tx
constexpr std::uint32_t isValid{0x08000002}; //dest done tx


uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, bool verbose = false, const int ms_delay = 1);
void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, const int ms_delay = 1);
void read_sample_mem(uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const std::string file);
void start_tx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits);

#endif  // MMIO_H
