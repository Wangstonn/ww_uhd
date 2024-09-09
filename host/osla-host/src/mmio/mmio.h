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
    constexpr uint32_t kConfigAddr = 0x0000001;
    constexpr uint64_t kSrcDelayAddr = 0x80000011;
    constexpr uint32_t kSrcTxAmpAddr = 0x12;
    constexpr uint32_t kMaxSrcTxAmp = 0x7FFF; //	UINT_MAX
    constexpr uint32_t kSrcThreshold = 0x10;
    

    constexpr uint64_t kDestDelayAddr = 0x80000033;
    constexpr uint64_t kDestNumBitShift = 0x80000034;

    constexpr uint32_t kDestChipCapEn = 0x35;

    //Compensation
    constexpr uint32_t kDestChEqReAddr = 0x31;
    constexpr uint32_t kDestChEqImAddr = 0x32;

    const int kSrcThresholdFrac = 5;
    
    constexpr int kPktLen = 256;
    constexpr uint32_t kInPktAddr = 0x20;
    constexpr uint32_t kOutPktAddr = 0x810;

    //Useful read commands
    //constexpr std::uint32_t isDone{0x08000001}; //src done tx
    constexpr int kCapIdxWidth = 16;
    constexpr uint32_t kCapIdxMask = std::pow(2,kCapIdxWidth)-1;

    constexpr std::uint32_t kBbStatusAddr{0x800}; 
    constexpr std::uint32_t kSrcCapIdxAddr{0x819};
    constexpr std::uint32_t kDestCapIdxAddr{0x820}; 

    //Sample Capture Memory Address limits
    constexpr std::uint32_t kCapMaxNumSamps = std::pow(2,16); //RTL BUGGED SO IT CAN ONLY READ std::pow(2,16)-1 SAMPLES

    const uint32_t kSrcCapStartAddr = 0x01000000;
    const uint32_t kSrcCapEndAddr = kSrcCapStartAddr+kCapMaxNumSamps-1; //0x0100FFFF;

    const uint32_t kDestCapStartAddr = 0x02000000;
    const uint32_t kDestCapEndAddr = kDestCapStartAddr+kCapMaxNumSamps-1;// 0x0200FFFF;

    const uint32_t kSyncCtrAddr = 0x801;


    uint32_t RdMmio(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t addr, bool verbose = false);
    void WrMmio(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t addr, const uint32_t data);
    void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint64_t cmd);
    std::vector<std::complex<double>> ReadSampleMem(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps = pow(2,12), const std::string& file = "");
    std::vector<double> ReadChipMem(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps = pow(2,12), const std::string& file = "");
    void StartTx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits);
    void InitBBCore (uhd::usrp::multi_usrp::sptr tx_usrp);
    void ReadBBCore (uhd::usrp::multi_usrp::sptr tx_usrp);
    void ClearAddrBuffer (uhd::usrp::multi_usrp::sptr tx_usrp);

    void P2PStartTxRx(uhd::usrp::multi_usrp::sptr src_tx_usrp, uhd::usrp::multi_usrp::sptr dest_tx_usrp, std::uint32_t mode_bits, std::uint32_t gpio_start_sel_bits, uint32_t fix_len_mode_bits, std::uint32_t start_sync_mode_bit, const bool skip_rst);
}

#endif  // MMIO_H
