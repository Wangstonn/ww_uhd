#include "mmio.h"
#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

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

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
    
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
        std::cout << "command: " << std::hex << std::setw(8) << std::setfill('0') << readback_cmd << " data: " << std::hex << std::setw(8) << std::setfill('0') << data << std::endl; 

    return data;
}

/**
 * Writes to the mmio memory. Since the data written is typically quite small, just use the sleep even though its slow
*/
void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint64_t cmd)
{
    if(cmd >> 63 == 0) //check to make sure cmd is a wr command
        std::cout << std::hex << std::setw(4) << std::setfill('0') << "WARNING: Read command use where write command was expected. cmd: " << cmd << std::endl;

    tx_usrp->set_gpio_attr("FP0A", "OUT", static_cast<uint32_t>(cmd)); 
    tx_usrp->set_gpio_attr("FP0B", "OUT", static_cast<uint32_t>(cmd >> 32)); 
    std::this_thread::sleep_for(std::chrono::nanoseconds(5)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 
}




// void read_sample_mem(uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const std::string file){
  
//     std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

//     //speed test result: 10 mil samps -> 16 hrs
//     for(int i = 0; i < NumPrmblSamps; i++)
//     {
//         if (of_file.is_open()) {
//             uint32_t cmd = 0;
//             cmd  = (cmd & ~addrBits) | (PrmblStartAddr+i);
//             cmd  = (cmd & ~cmdBits) | (0<<31);
//             uint32_t prmbl_samp = static_cast<uint32_t>(rd_mem_cmd(tx_usrp, cmd));

//             uint16_t prmbl_samp_I, prmbl_samp_Q;
//             prmbl_samp_I = static_cast<uint16_t>(prmbl_samp >> 16); //shouldnt do sign extension
//             prmbl_samp_Q = static_cast<uint16_t>(prmbl_samp & 0x0000FFFF);
//             // Write the 32 bits to the file
//             of_file.write(reinterpret_cast<const char*>(&prmbl_samp_I), sizeof(prmbl_samp_I));
//             of_file.write(reinterpret_cast<const char*>(&prmbl_samp_Q), sizeof(prmbl_samp_Q));

//             // Write sample to vector
//             std::complex<double> samp = {static_cast<double>(prmbl_samp_I),static_cast<double>(prmbl_samp_Q)};
//             // Scale this to where we expect the "decimal point" is
//             samp *= static_cast<std::complex<double>>(pow(2, -8));
//             cap_samps.push_back(samp);
//             //std::cout << std::dec << i << " " << samp << std::endl;

//         } else {
//             std::cerr << "Unable to open file for appending." << std::endl;
//         }

//     }
    
//     // Close the file
//     of_file.close();

//     //std::cout << "Samples written to file" << std::endl;
    
// }

//Hardcode here because I dont anticipate switching between different mmios
const uint32_t PrmblStartAddr = 0x02000000;

/**
 * Loops through the MMIO and reads each of the samples. If there is a valid of_file object, then write to that file. 
*/
void read_samples_helper(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const int NumPrmblSamps, std::ofstream& of_file) {
    for(int i = 0; i < NumPrmblSamps; i++) {
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
        samp *= static_cast<std::complex<double>>(pow(2, -8));
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
 * @param NumPrmblSamps number of samples to be captured. Max is 2**15
 * @return void
 */
void read_sample_mem(const uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const int NumPrmblSamps = pow(2,12), const std::string& file = "") {
    if (!file.empty()) {
        std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

        if (!of_file.is_open()) {
            std::cerr << "Unable to open file for appending." << std::endl;
            return;
        }

        // Call the helper function for reading samples
        read_samples_helper(tx_usrp, cap_samps, NumPrmblSamps, of_file);

        // Close the file
        of_file.close();
    } else {
        // Call the helper function for reading samples without writing to the file
        std::ofstream of_file;
        //of_file.open("");
        read_samples_helper(tx_usrp, cap_samps, NumPrmblSamps, of_file);
    }
}


