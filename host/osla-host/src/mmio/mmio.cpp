#include "mmio.h"
#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

/*
Write a command to gpio-in and then reads the contents of gpio-out and prints it to the console 
*/
uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, bool verbose, const int ms_delay)
{
   if(cmd >> 31) //check to make sure cmd is a read command
        std::cout << "WARNING: Write command used where read command was expected. cmd: " << cmd << std::endl;

   tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
   std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); 

   uint32_t output_reg;
   output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK"); 
   std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay));

    if(verbose)
        std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl; 

   return output_reg;
}


void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, const int ms_delay)
{
    if(cmd >> 31 == 0) //check to make sure cmd is a wr command
        std::cout << "WARNING: Read command use where write command was expected. cmd: " << cmd << std::endl;

    tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); 
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
