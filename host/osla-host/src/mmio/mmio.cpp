#include "mmio.h"
#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

namespace mmio {

    /**
     *  Configures the runtime mode of the baseband core and initiates tx
     * 
     * @param mode_bits bb-engine mode: active (pkt tx), sync. 
     *      2 bits [src,dest]. For each, 1->active, 0->sync. ex: mode 3 =>both active
     * @param rx_ch_sel_bits controls whther the src/dest are listening to the ch-emu or afe
            2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
            ex: mode 0 =>both digital loopback
            ex: mode 1 =>dest rx to afe
            ex: mode 2 =>src rx to afe
            //1->fwd analog loopback
    * @param tx_core_bits controls which engine transmits through the afe
            2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
            ex: mode 0 =>both digital loopback
            ex: mode 1 =>dest afe tx
            ex: mode 2 =>src afe tx
            //2->fwd analog loopback
        @param gpio_start_sel_bits controls whether [src,dest] uses register start or gpio start
                2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
                ex: mode 0 =>both register start
                ex: mode 1 =>dest gpio start
                ex: mode 2 =>src gpio start
    */
    void start_tx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits) {
        std::uint32_t mode_bits_shift{mode_bits << 2};
        std::uint32_t rx_ch_sel_bits_shift{rx_ch_sel_bits << 4}; 
        std::uint32_t tx_core_bits_shift{tx_core_bits << 6}; 
        std::uint32_t gpio_start_sel_bits_shift{gpio_start_sel_bits << 8};

        uint64_t start_cmd = 0x80000001'00000002+mode_bits_shift+rx_ch_sel_bits_shift+tx_core_bits_shift+gpio_start_sel_bits_shift;
        //std::cout << "mode: " << mode_bits << " rxChSel: " << rx_ch_sel_bits << " txCore: " << tx_core_bits << " gpio_start_sel_bits: << gpio_start_sel_bits << std::endl; 
        //std::cout << std::hex << std::setw(8) << std::setfill('0') << start_data << std::endl;

        //write rst command to device in case we run this multiple times. I forgot this but acqusition went fine...
        wr_mem_cmd(tx_usrp, rst_cmd);

        wr_mem_cmd(tx_usrp,0x80000000'00000000); //need to clear addr buffer

        //rd_mem_cmd(tx_usrp,0x00000001,1);

        //std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
        
        //std::cout << "Start command issued...\n";
        wr_mem_cmd(tx_usrp, start_cmd);
        //rd_mem_cmd(tx_usrp,0x00000001,1);
        
        //should poll done until it flips to one, but this always seems to finish before its necessary. Also which bit to poll varies between test

        // std::cout << "Reading results...\n";
        // for(const auto& cmd : read_cmds) {
        //     rd_mem_cmd(tx_usrp, cmd,true);
        // }
        // std::cout << "Done printing digital loopback results...\n";
    }

    //Load phases and thresholds
    std::vector<uint64_t> write_cmds = {
        0x80000000'00000000,
        0x80000001'00000001,
        0x80000010'00001000,
        0x80000011'00000000,
        0x80000020'E12ACE94,
        0x80000021'E12ACE94,
        0x80000030'27100000,
        0x80000031'00002000,
        0x80000032'00000000,
        0x80000033'00000075,
        0x80000034'00007FFF
    };

    //Readback input bit, phase, and threshold settings, valid and done, and pkt out
    std::vector<uint32_t> read_cmds = {
        0x00000000,
        0x00000001,
        0x00000010,
        0x00000011,
        0x00000020,
        0x00000021,
        0x00000030,
        0x00000031,
        0x00000032,
        0x00000033,
        0x00000034,
        
        0x00000800,
        0x00000810,
        0x00000811,
        0x00000819,
        0x02000000,
        0x020003FF
    };

    void InitBBCore (uhd::usrp::multi_usrp::sptr tx_usrp) {
        for(const auto& cmd : write_cmds) {
            wr_mem_cmd(tx_usrp, cmd);
        }
    }

    void ReadBBCore (uhd::usrp::multi_usrp::sptr tx_usrp) {
        for(const auto& cmd : read_cmds) {
            rd_mem_cmd(tx_usrp, cmd,true);
        }
    }
        
    /**
    *  Write a command to gpio-in and then reads the contents of gpio-out and prints it to the console 
    */
    uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, bool verbose)
    {
    if(cmd >> 31) //check to make sure cmd is a read command
            std::cout << "WARNING: Write command used where read command was expected. cmd: " << cmd << std::endl;

        uint32_t last_cmd;
        last_cmd = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 

        tx_usrp->set_gpio_attr("FP0A", "OUT", 0x0); 
        tx_usrp->set_gpio_attr("FP0B", "OUT", cmd); 
        //std::this_thread::sleep_for(std::chrono::nanoseconds(5)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 


        uint32_t readback_cmd, data;
        readback_cmd = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 
        data = tx_usrp->get_gpio_attr("FP0A", "READBACK"); 

        if (last_cmd == readback_cmd) {
            readback_cmd = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 
            std::cerr << "ERROR: rd_mem_cmd readback_cmd = last_cmd" << std::endl;
        }
        // std::this_thread::sleep_for(std::chrono::nanoseconds(5));

        if(verbose)
            std::cout << "Addr: " << std::hex << std::setw(8) << std::setfill('0') << readback_cmd << " data: " << std::hex << std::setw(8) << std::setfill('0') << data << std::endl; 

        return data;
    }

    /**
     * @brief Writes a memory command to the specified USRP device.
     * 
     * This function sets GPIO attributes to write data to the MMIO. It warns if a read command is used instead of a write command. 
     * 
     * @param tx_usrp A shared pointer to the USRP device for transmission.
     * @param cmd Command and data to be written. 64 bit {command, data}
     * 
     * @details
     * The function sets GPIO attributes "FP0A" and "FP0B" of the USRP device to transmit
     * the lower 32 bits and upper 32 bits of the memory command, respectively. Additionally,
     * it warns if the most significant bit of the command is 0, indicating a read command
     * where a write command was expected.
     */
    void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint64_t cmd)
    {
        if(cmd >> 63 == 0) //check to make sure cmd is a wr command
            std::cout << std::hex << "WARNING: Read command use where write command was expected. cmd: " << std::setw(16) << std::setfill('0') << cmd << std::endl; //setw and setfill make sure preceding zeros are captured

        tx_usrp->set_gpio_attr("FP0A", "OUT", static_cast<uint32_t>(cmd)); 
        tx_usrp->set_gpio_attr("FP0B", "OUT", static_cast<uint32_t>(cmd >> 32)); 
        std::this_thread::sleep_for(std::chrono::nanoseconds(5)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 
    }

    /**
     * @brief Write data to an MMIO address
     * 
     * This is a wrapper for wr_mem_cmd to make forming commands easier
    */
    void WrMmio(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t addr, const uint32_t data)
    {
        uint64_t cmd = 0;
        cmd  = (cmd & ~addrBits) | (static_cast<uint64_t>(addr)<<32);
        cmd  = (cmd & ~dataBits) | (data<<0);
        cmd  = (cmd & ~cmdBits) | (static_cast<uint64_t>(0x1)<<63);
        wr_mem_cmd(tx_usrp, cmd);
    }


    //Hardcode here because I dont anticipate switching between different mmios
    const uint32_t PrmblStartAddr = 0x02000000;

    /**
     * Reads the captured samples from the MMIO and writes them to a vector of complex doubles
     * and/or to an output file.
     * 
     * @param tx_usrp The USRP device to read samples from.
     * @param cap_samps Reference to a vector of complex doubles where the samples will be stored.
     * @param frac_len Number of bits used to represent the fractional part of the number. This depends on RX implementation!!
     * @param NCapSamps The number of samples to read.
     * @param of_file Reference to an ofstream object representing the output file.
     *                If this file is open, the samples will be written to it.
     * @return None
     *
     * This function reads NCapSamps samples from the specified USRP device.
     * For each sample, it constructs a 16-bit complex value from two 16-bit integers
     * (one for the real part and one for the imaginary part), and stores it in the 
     * cap_samps vector. If an output file is provided and open, each 16-bit integer 
     * representing the real and imaginary parts of the sample is written to the file.
     * The samples are converted to complex doubles and normalized by multiplying
     * by 2^-frac_len before being stored.
     */
    void read_samples_helper(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const int frac_len, const int NCapSamps, std::ofstream& of_file) {
        for(int i = 0; i < NCapSamps; i++) {
            uint32_t cmd = 0;
            cmd  = (cmd & ~addrBits) | (PrmblStartAddr+i);
            cmd  = (cmd & ~cmdBits) | (0<<31);
            uint32_t prmbl_samp = static_cast<uint32_t>(rd_mem_cmd(tx_usrp, cmd));

            int16_t prmbl_samp_I, prmbl_samp_Q; //integer because samples are signed
            prmbl_samp_I = static_cast<int16_t>(prmbl_samp >> 16);
            prmbl_samp_Q = static_cast<int16_t>(prmbl_samp & 0x0000FFFF);

            // Write the 16 bits to the file if ofstream is open
            if (of_file.is_open()) {
                //write takes a char pointer and the number of bytes to write. As a result, we need to typecast and take sizeof
                of_file.write(reinterpret_cast<const char*>(&prmbl_samp_I), sizeof(prmbl_samp_I));
                of_file.write(reinterpret_cast<const char*>(&prmbl_samp_Q), sizeof(prmbl_samp_Q));
            }

            // Write sample to vector
            std::complex<double> samp = {static_cast<double>(prmbl_samp_I), static_cast<double>(prmbl_samp_Q)};
            samp *= static_cast<std::complex<double>>(pow(2, -frac_len));
            cap_samps.push_back(samp);
        }
    }

    /**
     * Reads the MMIO captured sample data to file or vector.
     * The MMIO is a very low bandwidth interface, so this is only suitable for small amounts of capture 
     * It would take XX hrs to read 10 million samples. Its good enough for a thousand samples or so. Otherwise streaming strobed data is recommended
     *
     * @param file Name of file to write to. If empty/missing will not write to file.
     * @param cap_samps vector that will be cleared and filled with captured IQ samples
     * @param NCapSamps number of samples to be captured. Max is 2**15
     * @return void
     */
    void read_sample_mem(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const int NCapSamps = pow(2,12), const std::string& file = "") {
        const int frac_len = 6; 
        if (!file.empty()) {
            std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

            if (!of_file.is_open()) {
                std::cerr << "Unable to open file for appending." << std::endl;
                return;
            }

            // Call the helper function for reading samples
            read_samples_helper(tx_usrp, cap_samps, frac_len, NCapSamps, of_file);

            // Close the file
            of_file.close();
        } else {
            // Call the helper function for reading samples without writing to the file
            std::ofstream of_file;
            //of_file.open("");
            read_samples_helper(tx_usrp, cap_samps, frac_len, NCapSamps, of_file);
        }
    }

}
