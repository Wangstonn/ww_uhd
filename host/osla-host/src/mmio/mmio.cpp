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
 * @param modeBits bb-engine mode: active (pkt tx), sync. 
 *      2 bits [src,dest]. For each, 1->active, 0->sync. ex: mode 3 =>both active
 * @param rxChSelBits controls whther the src/dest are listening to the ch-emu or afe
        2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
        ex: mode 0 =>both digital loopback
        ex: mode 1 =>dest rx to afe
        ex: mode 2 =>src rx to afe
        //1->fwd analog loopback
 * @param txCoreBits controls which engine transmits through the afe
        2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
        ex: mode 0 =>both digital loopback
        ex: mode 1 =>dest afe tx
        ex: mode 2 =>src afe tx
        //2->fwd analog loopback
*/
void start_tx(uhd::usrp::multi_usrp::sptr tx_usrp, std::uint32_t mode_bits, std::uint32_t rx_ch_sel_bits, std::uint32_t tx_core_bits) {
    std::uint32_t mode_bits_shift{mode_bits << 2};
    std::uint32_t rx_ch_sel_bits_shift{rx_ch_sel_bits << 4}; 
    std::uint32_t tx_core_bits_shift{tx_core_bits << 6}; 

    uint32_t start_cmd = 0x80010002+mode_bits_shift+rx_ch_sel_bits_shift+tx_core_bits_shift;
    //std::cout << "mode: " << mode_bits << " rxChSel: " << rx_ch_sel_bits << " txCore: " << tx_core_bits << std::endl; 
    //std::cout << std::hex << std::setw(8) << std::setfill('0') << start_cmd << std::endl;

    //write rst command to device in case we run this multiple times. I forgot this but acqusition went fine...
    wr_mem_cmd(tx_usrp, rst_cmd);

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
    
    //std::cout << "Start command issued...\n";
    wr_mem_cmd(tx_usrp, start_cmd);
    
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
uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, bool verbose, const int ms_delay)
{
   if(cmd >> 31) //check to make sure cmd is a read command
        std::cout << "WARNING: Write command used where read command was expected. cmd: " << cmd << std::endl;

    uint32_t last_output;
    last_output = tx_usrp->get_gpio_attr("FP0", "READBACK"); 

    tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
    //std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 


    uint32_t output_reg;
    output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK"); 

    if (last_output == output_reg)
        std::cerr << "ERROR: rd_mem_cmd output_reg = last_output" << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay));

    if(verbose)
        std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl; 

    return output_reg;
}

/**
 * Writes to the mmio memory. Since the data written is typically quite small, just use the sleep even though its slow
*/
void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, const int ms_delay)
{
    if(cmd >> 31 == 0) //check to make sure cmd is a wr command
        std::cout << "WARNING: Read command use where write command was expected. cmd: " << cmd << std::endl;

    tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 
}



/**
 * Reads the MMIO captured sample data to file or vector.
 * The MMIO is a very low bandwidth interface, so this is only suitable for small amounts of capture 
 * It would take 16 hrs to read 10 million samples. Its good enough for a thousand samples or so. Otherwise streaming strobed data is recommended
 *
 * @param file Name of file to write to. 
 * @param cap_samps vector that will be cleared and filled with captured IQ samples
 * 
 * @return void
 */

//Hardcode here because I dont anticipate switching between different mmios
const int NumPrmblSamps = 512;
const uint16_t PrmblStartAddrI = 0xC00, PrmblStartAddrQ = 0xE00;

void read_sample_mem(uhd::usrp::multi_usrp::sptr tx_usrp, std::vector<std::complex<double>>& cap_samps, const std::string file){
  
    std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

    //speed test result: 10 mil samps -> 16 hrs
    for(int i = 0; i < NumPrmblSamps; i++)
    {
        if (of_file.is_open()) {
            uint32_t cmd = 0;
            cmd  = (cmd & ~addrBits) | (PrmblStartAddrI+i<<16);
            cmd  = (cmd & ~dataBits) | (0<<0);
            cmd  = (cmd & ~cmdBits) | (0<<31);
            int16_t prmbl_samp_I = static_cast<int16_t>(rd_mem_cmd(tx_usrp, cmd) & dataBits);

            // Write the last 16 bits to the file
            of_file.write(reinterpret_cast<const char*>(&prmbl_samp_I), sizeof(prmbl_samp_I));

            cmd = 0;
            cmd  = (cmd & ~addrBits) | (PrmblStartAddrQ+i<<16);
            cmd  = (cmd & ~dataBits) | (0<<0);
            cmd  = (cmd & ~cmdBits) | (0<<31);
            int16_t prmbl_samp_Q = static_cast<int16_t>(rd_mem_cmd(tx_usrp, cmd) & dataBits);
            // Write the last 16 bits to the file
            of_file.write(reinterpret_cast<const char*>(&prmbl_samp_Q), sizeof(prmbl_samp_Q));
        
            // Write sample to vector
            std::complex<double> samp = {static_cast<double>(prmbl_samp_I),static_cast<double>(prmbl_samp_Q)};
            // Scale this to where we expect the "decimal point" is
            samp *= static_cast<std::complex<double>>(pow(2, -8));
            cap_samps.push_back(samp);
            //std::cout << std::dec << i << " " << samp << std::endl;

        } else {
            std::cerr << "Unable to open file for appending." << std::endl;
        }

    }
    
    // Close the file
    of_file.close();

    //std::cout << "Samples written to file" << std::endl;
    


}
