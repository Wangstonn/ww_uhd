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
    void StartTx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits) {
        std::uint32_t mode_bits_shift{mode_bits << 2};
        std::uint32_t rx_ch_sel_bits_shift{rx_ch_sel_bits << 4}; 
        std::uint32_t tx_core_bits_shift{tx_core_bits << 6}; 
        std::uint32_t gpio_start_sel_bits_shift{gpio_start_sel_bits << 8};

        uint64_t start_cmd = 0x80000001'00000002+mode_bits_shift+rx_ch_sel_bits_shift+tx_core_bits_shift+gpio_start_sel_bits_shift;
        //std::cout << "mode: " << mode_bits << " rxChSel: " << rx_ch_sel_bits << " txCore: " << tx_core_bits << " gpio_start_sel_bits: << gpio_start_sel_bits << std::endl; 
        //std::cout << std::hex << std::setw(8) << std::setfill('0') << start_data << std::endl;

        //write rst command to device in case we run this multiple times. I forgot this but acqusition went fine...
        wr_mem_cmd(tx_usrp, rst_cmd);

        tx_usrp->set_rx_dc_offset(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(40)); //DC offset calibration time, minimum is 33.6 ms
        tx_usrp->set_rx_dc_offset(false);

        wr_mem_cmd(tx_usrp,0x80000000'00000000); //need to clear addr buffer, not sure why its 0x8. 0x0 should work fine...

        //RdMmio(tx_usrp,0x00000001,1);

        //std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
        
        //std::cout << "Start command issued...\n";
        wr_mem_cmd(tx_usrp, start_cmd);
        //RdMmio(tx_usrp,0x00000001,1);
        
        //should poll done until it flips to one, but this always seems to finish before its necessary. Also which bit to poll varies between test

        // std::cout << "Reading results...\n";
        // for(const auto& cmd : read_cmds) {
        //     RdMmio(tx_usrp, cmd,true);
        // }
        // std::cout << "Done printing digital loopback results...\n";
    }

    uint32_t MakeStartCmd(std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits, std::uint32_t gpio_start_sel_bits) {
        std::uint32_t mode_bits_shift{mode_bits << 2};
        std::uint32_t rx_ch_sel_bits_shift{rx_ch_sel_bits << 4}; 
        std::uint32_t tx_core_bits_shift{tx_core_bits << 6}; 
        std::uint32_t gpio_start_sel_bits_shift{gpio_start_sel_bits << 8};

        std::uint32_t config_data = mode_bits_shift+rx_ch_sel_bits_shift+tx_core_bits_shift+gpio_start_sel_bits_shift;

        return config_data;
    }

    const std::uint32_t kP2PSrcTxCoreBits = 0b10; //connect afe to src module
    const std::uint32_t kP2PSrcRxChSelBits = 0b10;
    const std::uint32_t kP2PDestTxCoreBits = 0b01; //connect afe to dest module
    const std::uint32_t kP2PDestRxChSelBits = 0b01;
    /**
     *  Configures the runtime mode of the baseband core for p2p communications and initiates it.
     * There are much less params since There is only one real possible configuration for p2p comms
     * 
     * @param mode_bits bb-engine mode: active (pkt tx), sync. 
     *      2 bits [src,dest]. For each, 1->active, 0->sync. ex: mode 3 =>both active
    */
    void P2PStartTxRx(uhd::usrp::multi_usrp::sptr src_tx_usrp, uhd::usrp::multi_usrp::sptr dest_tx_usrp, std::uint32_t mode_bits, std::uint32_t gpio_start_sel_bits) {
        uint32_t SrcStartCmd = MakeStartCmd(mode_bits, kP2PSrcRxChSelBits, kP2PSrcTxCoreBits, gpio_start_sel_bits) + 0x2; //0x2 needed for forward
        uint32_t DestStartCmd = MakeStartCmd(mode_bits, kP2PDestRxChSelBits, kP2PDestTxCoreBits, gpio_start_sel_bits) + 0x2; // 0x2 needed for feedback. If gpio start is set, then this start will be ignored

        //Reset device
        WrMmio(src_tx_usrp, kConfigAddr, 0x0000001);
        WrMmio(dest_tx_usrp, kConfigAddr, 0x0000001);
        std::this_thread::sleep_for(std::chrono::nanoseconds(5*5)); //Leave the reset for a couple of cycles

        //Calibrate DC Offset
        src_tx_usrp->set_rx_dc_offset(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(40)); //DC offset calibration time, minimum is 33.6 ms
        src_tx_usrp->set_rx_dc_offset(false);

        ClearAddrBuffer(src_tx_usrp);
        ClearAddrBuffer(dest_tx_usrp);

        //std::cout << "Start command issued...\n";
        WrMmio(dest_tx_usrp, kConfigAddr, DestStartCmd);
        std::this_thread::sleep_for(std::chrono::nanoseconds(5*5)); //Leave the reset for a couple of cycles
        WrMmio(src_tx_usrp, kConfigAddr, SrcStartCmd);
        
    }

    //Load phases and thresholds
    std::vector<uint64_t> write_cmds = {
        0x80000000'00000000,
        0x80000001'00000001,
        0x80000010'00001000,
        0x80000011'00000000,
        0x80000012'00007FFF,
        0x80000020'E12ACE94,
        0x80000021'E12ACE94,
        0x80000030'27100000,
        0x80000031'00002000,
        0x80000032'00000000,
        0x80000033'00000075,
        0x80000034'00000000,
        0x80000035'00000000
    };

    //Readback input bit, phase, and threshold settings, valid and done, and pkt out
    std::vector<uint32_t> read_cmds = {
        0x00000000,
        0x00000001,
        0x00000010,
        0x00000011,
        0x00000012,
        0x00000020,
        0x00000021,
        0x00000030,
        0x00000031,
        0x00000032,
        0x00000033,
        0x00000034,
        0x00000035,
        
        0x00000800,
        0x00000810,
        0x00000811,

        0x00000819, //src samp cap counter
        0x00000820, //dest samp cap counter

        0x01000000, //src samp cap memory
        0x0100FFFF,
        0x02000000, //dest samp cap memory
        0x02002100,
        0x0200FFFF
    };

    void InitBBCore (uhd::usrp::multi_usrp::sptr tx_usrp) {
        for(const auto& cmd : write_cmds) {
            wr_mem_cmd(tx_usrp, cmd);
        }
        mmio::ClearAddrBuffer(tx_usrp); //reset for the next cmd
    }

    void ReadBBCore (uhd::usrp::multi_usrp::sptr tx_usrp) {
        for(const auto& cmd : read_cmds) {
            RdMmio(tx_usrp, cmd,true);
        }
        mmio::ClearAddrBuffer(tx_usrp);
    }
    
    const uint32_t kAddrMask = 0x0FFFFFFF;
    /**
     * Writes a command to the GPIO input and then reads the contents of the GPIO output,
     * printing it to the console.
     * 
     * @param tx_usrp The USRP device to perform the GPIO operations.
     * @param addr The address or memory to be accessed. Should be a 28 bit number.
     * @param verbose Boolean indicating whether to print verbose output.
     * @return The data read from the GPIO output.
     * 
     * This function writes a command to the specified address in the GPIO input and then reads
     * the corresponding data from the GPIO output. It prints the address and data to the console
     * if the `verbose` flag is set to true. If a write command is detected where a read command
     * was expected, a warning message is printed to the standard error stream. It also checks if 
     * the address was valid by checking that the fpga read back the address
     */
    uint32_t RdMmio(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t addr, bool verbose)
    {
        uint32_t read_addr = addr & kAddrMask;
        if (addr &  !kAddrMask) {
            // Throw an error indicating that the variable exceeds 24 bits
            std::cerr << "Error: 28-bit address expected. But read command had more than 28 bits." << std::endl;
        }

        uint32_t last_addr;
        last_addr = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 

        tx_usrp->set_gpio_attr("FP0A", "OUT", 0x0); 
        tx_usrp->set_gpio_attr("FP0B", "OUT", read_addr); 
        //std::this_thread::sleep_for(std::chrono::nanoseconds(5)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 


        uint32_t readback_addr, data;
        readback_addr = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 
        data = tx_usrp->get_gpio_attr("FP0A", "READBACK"); 

        if (last_addr == readback_addr) {
            readback_addr = tx_usrp->get_gpio_attr("FP0B", "READBACK"); 
            std::cerr << "ERROR: RdMmio readback_addr = last_addr" << std::endl;
        }
        // std::this_thread::sleep_for(std::chrono::nanoseconds(5));

        if(verbose)
            std::cout << "Addr: " << std::hex << std::setw(8) << std::setfill('0') << readback_addr << " data: " << std::hex << std::setw(8) << std::setfill('0') << data << std::endl; 

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
            std::cerr << std::hex << "WARNING: Read command use where write command was expected. cmd: " << std::setw(16) << std::setfill('0') << cmd << std::endl; //setw and setfill make sure preceding zeros are captured

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

    void ClearAddrBuffer (uhd::usrp::multi_usrp::sptr tx_usrp) {
        WrMmio(tx_usrp,0,0); //need to clear addr buffer, not sure why its 0x8. 0x0 should work fine...
    }
    


    //Sample Capture Memory Address limits
    const uint32_t kSrcCapStartAddr = 0x01000000;
    const uint32_t kSrcCapEndAddr = 0x0100FFFF;

    const uint32_t kDestCapStartAddr = 0x02000000;
    const uint32_t kDestCapEndAddr = 0x0200FFFF;
    //Warning: in some cases, capture will not fully fill the memory. It is important to verify that the index being read is less than DestCapIdx.
    // otherwise, UNINTIALIZED memory will be read, given nonsensical results. Futhermore, verify that the device has completed operation. Otherwise,
    // the output may be the current memory value being written!

    /**
     * Reads the captured samples from the MMIO and writes them to a vector of complex doubles
     * and/or to an output file.
     * 
     * @param tx_usrp The USRP device to read samples from.
     * @param mem_sel Selects which memory to read from. 0->src, 1->dest
     * @param NCapSamps The number of samples to read.
     * @param frac_len Number of bits used to represent the fractional part of the number. This depends on RX implementation!!
     * @param of_file Reference to an ofstream object representing the output file.
     *                If this file is open, the samples will be written to it.
     * @return A vector of complex doubles containing the captured samples.
     *
     * This function reads NCapSamps samples from the specified USRP device.
     * For each sample, it constructs a 16-bit complex value from two 16-bit integers
     * (one for the real part and one for the imaginary part), and stores it in the 
     * cap_samps vector. If an output file is provided and open, each 16-bit integer 
     * representing the real and imaginary parts of the sample is written to the file.
     * The samples are converted to complex doubles and normalized by multiplying
     * by 2^-frac_len before being stored.
     * If the specified number of samples exceeds the available memory address range,
     * the function reads the maximum allowed number of samples and prints an error message.
     * 
     * !TODO: This does not check if capture is finished! Need FPGA work to signal when cap is done. Not sure actually. Check might be context dependent.
     */
    std::vector<std::complex<double>> ReadSampleHelper(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps, const int frac_len, std::ofstream& of_file) {
        
        std::vector<std::complex<double>> cap_samps; // Define a vector to store the samples
        
        uint32_t start_addr = mem_sel ? kDestCapStartAddr : kSrcCapStartAddr;
        uint32_t end_addr = start_addr + NCapSamps - 1; // Calculate the end address
        uint32_t max_end_addr = mem_sel ? kDestCapEndAddr : kSrcCapEndAddr;

        uint32_t idx_addr = mem_sel ? mmio::kDestCapIdxAddr : mmio::kSrcCapIdxAddr;
        // Check if the end address exceeds the maximum address
        if (end_addr > max_end_addr) {
            std::cerr << "Warning: Address range exceeds maximum allowed address. Reading maximum allowed instead." << std::endl;
            end_addr = max_end_addr;
        }

        // Check how many sample were written and only read those
        end_addr = std::min(start_addr + mmio::RdMmio(tx_usrp, idx_addr)-1,end_addr); //samp_cap_idx displays the last written memory address. Seems to be bugged but I cant find the problem

        for(uint32_t addr = start_addr; addr <= end_addr; addr++) {
            uint32_t prmbl_samp = RdMmio(tx_usrp, addr);
            //std::cout<< std::hex << "addr: " << addr << " data: " << prmbl_samp << std::endl;

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
        return cap_samps; // Return the vector containing the samples
    }

    /**
     * Reads the MMIO captured sample data to file or vector.
     * The MMIO is a very low bandwidth interface, so this is only suitable for small amounts of capture 
     * It would take XX hrs to read 10 million samples. Its good enough for a thousand samples or so. Otherwise streaming strobed data is recommended
     *
     * @param tx_usrp The USRP device to read samples from.
     * @param mem_sel Selects which memory to read from. 0->src, 1->dest
     * @param NCapSamps number of samples to be captured. Max is 2**16
     * @param file Name of file to write to. If empty/missing will not write to file.
     * @return A vector of complex doubles containing the captured samples.
     */
    std::vector<std::complex<double>> ReadSampleMem(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps , const std::string& file) {
        //Want the double equivalent to the fixed point numbers seen
        const int kDestFracLen = 6; //(14,4)
        const int kSrcFracLen = 12; //(14,10)

        int frac_len = mem_sel ? kDestFracLen : kSrcFracLen;

        std::vector<std::complex<double>> cap_samps; // Define a vector to store the samples

        if (!file.empty()) {
            std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

            if (!of_file.is_open()) {
                std::cerr << "Error: ReadSampleMem Unable to open file for writing." << std::endl;
                return cap_samps; //return empty vector if file cannot be opened
            }

            // Call the helper function for reading samples
            cap_samps = ReadSampleHelper(tx_usrp, mem_sel, NCapSamps, frac_len, of_file);
            std::cout << "Samples written to " << file << std::endl; 

            // Close the file
            of_file.close();
        } else {
            // Call the helper function for reading samples without writing to the file
            std::ofstream of_file;
            cap_samps = ReadSampleHelper(tx_usrp, mem_sel, NCapSamps, frac_len, of_file);
        }

        return cap_samps; // Return the vector containing the samples
    }


    /**
     * Reads the captured samples from the MMIO and writes them to a vector of complex doubles
     * and/or to an output file.
     * 
     * @param tx_usrp The USRP device to read samples from.
     * @param mem_sel Selects which memory to read from. 0->src, 1->dest !WW Not implemented yet!
     * @param NCapSamps The number of samples to read.
     * @param frac_len Number of bits used to represent the fractional part of the number. This depends on RX implementation!!
     * @param of_file Reference to an ofstream object representing the output file.
     *                If this file is open, the samples will be written to it.
     * @return A vector of complex doubles containing the captured samples.
     *
     * This function reads NCapSamps samples from the specified USRP device.
     * For each sample, it constructs a 32-bit complex value from two 16-bit integers
     * (one for the real part and one for the imaginary part), and stores it in the 
     * cap_samps vector. If an output file is provided and open, each 16-bit integer 
     * representing the real and imaginary parts of the sample is written to the file.
     * The samples are converted to complex doubles and normalized by multiplying
     * by 2^-frac_len before being stored.
     * If the specified number of samples exceeds the available memory address range,
     * the function reads the maximum allowed number of samples and prints an error message.
     * 
     * !TODO: This does not check if capture is finished! Need FPGA work to signal when cap is done
     */
    std::vector<double> ReadChipHelper(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps, const int frac_len, std::ofstream& of_file) {
        
        std::vector<double> cap_chips; // Define a vector to store the samples
        
        uint32_t start_addr = mem_sel ? kDestCapStartAddr : kSrcCapStartAddr;
        uint32_t end_addr = start_addr + NCapSamps - 1; // Calculate the end address
        uint32_t max_end_addr = mem_sel ? kDestCapEndAddr : kSrcCapEndAddr;

        uint32_t idx_addr = mem_sel ? mmio::kDestCapIdxAddr : mmio::kSrcCapIdxAddr;
        // Check if the end address exceeds the maximum address
        if (end_addr > max_end_addr) {
            std::cerr << "Warning: Address range exceeds maximum allowed address. Reading maximum allowed instead." << std::endl;
            end_addr = max_end_addr;
        }

        // Check how many sample were written and only read those
        end_addr = std::min(start_addr + mmio::RdMmio(tx_usrp, mmio::kDestCapIdxAddr)-1,end_addr); //samp_cap_idx displays the last written memory address. Seems to be bugged but I cant find the problem

        for(uint32_t addr = start_addr; addr <= end_addr; addr++) {
            uint32_t prmbl_samp = RdMmio(tx_usrp, addr);
            //std::cout<< std::hex << "addr: " << addr << " data: " << prmbl_samp << std::endl;

            //legacy code to write I and Q to file
            int16_t prmbl_samp_I, prmbl_samp_Q; //integer because samples are signed
            prmbl_samp_I = static_cast<int16_t>(prmbl_samp >> 16);
            prmbl_samp_Q = static_cast<int16_t>(prmbl_samp & 0x0000FFFF);

            int32_t chip_int32 = prmbl_samp;

            // Write the 16 bits to the file if ofstream is open
            if (of_file.is_open()) {
                //write takes a char pointer and the number of bytes to write. As a result, we need to typecast and take sizeof
                of_file.write(reinterpret_cast<const char*>(&prmbl_samp_I), sizeof(prmbl_samp_I));
                of_file.write(reinterpret_cast<const char*>(&prmbl_samp_Q), sizeof(prmbl_samp_Q));
            }

            // Write sample to vector
            double chip = chip_int32 * pow(2, -frac_len) ;
            cap_chips.push_back(chip);
        }
        return cap_chips; // Return the vector containing the samples
    }

    /**
     * Read Chip capture memory. When capture memory is set to capture chip values, the data is in a real 32 bit form rather than complex 16 bit form. 
     * 
     * @param tx_usrp The USRP device to read samples from.
     * @param mem_sel Selects which memory to read from. 0->src, 1->dest !!!Not implemented yet!
     * @param NCapSamps number of samples to be captured. Max is 2**16
     * @param file Name of file to write to. If empty/missing will not write to file.
     * @return A vector of complex doubles containing the captured samples.
     */
    std::vector<double> ReadChipMem(const uhd::usrp::multi_usrp::sptr tx_usrp, const bool mem_sel, const int NCapSamps , const std::string& file) {
        const int ACCUM_WIDTH = 42;
        const int ACCUM_FRAC = 16;
        const int CAP_WIDTH = 32;
        const int kDestChipFracLen = ACCUM_FRAC-(ACCUM_WIDTH-32); //17 is accum frac, 40 is accum width, 32 is capture width, truncation is performed
        
        int frac_len = kDestChipFracLen;

        std::vector<double> cap_chips; // Define a vector to store the samples

        if (!file.empty()) {
            std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

            if (!of_file.is_open()) {
                std::cerr << "Error: ReadChipMem Unable to open file for writing." << std::endl;
                return cap_chips; //return empty vector if file cannot be opened
            }

            // Call the helper function for reading samples
            cap_chips = ReadChipHelper(tx_usrp, mem_sel, NCapSamps, frac_len, of_file);
            std::cout << "Samples written to " << file << std::endl; 

            // Close the file
            of_file.close();
        } else {
            // Call the helper function for reading samples without writing to the file
            std::ofstream of_file;
            cap_chips = ReadChipHelper(tx_usrp, mem_sel, NCapSamps, frac_len, of_file);
        }

        return cap_chips; // Return the vector containing the samples
    }
}
