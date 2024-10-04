//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
//WW-This file is a modified version of uhd/host/examples/txrx_loopback_to_file.cpp. It essentially sets up + activates the radios and writes & sends samples to file

/**
 * Control 2 USRPs to perform channel estimation for p2p communications.
 * In this case, the 2 usrp's time is synchronized using a single gpio signal. Since they are connected to the same external clock source,
 * no drift should occur.
 */

#include "../wavetable.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>
#include <random>
#include <string>

#include "../src/mmio/mmio.h"
#include "../src/estim/estim.h"

namespace po = boost::program_options;

/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

/***********************************************************************
 * Utilities
 **********************************************************************/
//! Change to filename, e.g. from usrp_samples.dat to usrp_samples.00.dat,
//  but only if multiple names are to be generated.
std::string generate_out_filename(
    const std::string& base_fn, size_t n_names, size_t this_name)
{
    if (n_names == 1) {
        return base_fn;
    }

    boost::filesystem::path base_fn_fp(base_fn);
    base_fn_fp.replace_extension(boost::filesystem::path(
        str(boost::format("%02d%s") % this_name % base_fn_fp.extension().string())));
    return base_fn_fp.string();
}


/***********************************************************************
 * transmit_worker function
 * A function to be used in a thread for transmitting
 **********************************************************************/
void transmit_worker(std::vector<std::complex<float>> buff,
    wave_table_class wave_table,
    uhd::tx_streamer::sptr tx_streamer,
    uhd::tx_metadata_t metadata,
    size_t step,
    size_t index,
    int num_channels)
{
    std::vector<std::complex<float>*> buffs(num_channels, &buff.front());

    // send data until the signal handler gets called
    while (not stop_signal_called) {
        // fill the buffer with the waveform
        for (size_t n = 0; n < buff.size(); n++) {
            buff[n] = wave_table(index += step);
        }

        // send the entire contents of the buffer
        tx_streamer->send(buffs, buff.size(), metadata);

        metadata.start_of_burst = false;
        metadata.has_time_spec  = false;
    }

    // send a mini EOB packet
    metadata.end_of_burst = true;
    tx_streamer->send("", 0, metadata);
}


/***********************************************************************
 * recv_to_file function
 **********************************************************************/
template <typename samp_type>
void recv_to_file(uhd::usrp::multi_usrp::sptr usrp,
    const std::string& cpu_format,
    const std::string& wire_format,
    const std::string& file,
    size_t samps_per_buff,
    int num_requested_samples,
    double settling_time,
    std::vector<size_t> rx_channel_nums,
    bool save_rx = false)
{
    int num_total_samps = 0;
    // create a receive streamer
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    stream_args.channels             = rx_channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args); //sets data format reg

    // Prepare buffers for received samples and metadata
    uhd::rx_metadata_t md;
    std::vector<std::vector<samp_type>> buffs(
        rx_channel_nums.size(), std::vector<samp_type>(samps_per_buff));
    // create a vector of pointers to point to each of the channel buffers
    std::vector<samp_type*> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++) {
        buff_ptrs.push_back(&buffs[i].front());
    }

    // Create one ofstream object per channel
    // (use shared_ptr because ofstream is non-copyable)
    std::vector<std::shared_ptr<std::ofstream>> outfiles;
    for (size_t i = 0; i < buffs.size(); i++) {
        const std::string this_filename = generate_out_filename(file, buffs.size(), i);
        outfiles.push_back(std::shared_ptr<std::ofstream>(
            new std::ofstream(this_filename.c_str(), std::ofstream::binary)));
    }
    UHD_ASSERT_THROW(outfiles.size() == buffs.size());
    UHD_ASSERT_THROW(buffs.size() == rx_channel_nums.size());
    bool overflow_message = true;
    // We increase the first timeout to cover for the delay between now + the
    // command time, plus 500ms of buffer. In the loop, we will then reduce the
    // timeout for subsequent receives.
    double timeout = settling_time + 0.5f;

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = usrp->get_time_now() + uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd); //no regs set

    while (not stop_signal_called
           and (num_requested_samples > num_total_samps or num_requested_samples == 0)) {
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samps_per_buff, md, timeout);
        timeout             = 0.1f; // small timeout for subsequent recv

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << "Timeout while streaming" << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (overflow_message) {
                overflow_message = false;
                std::cerr
                    << boost::format(
                           "Got an overflow indication. Please consider the following:\n"
                           "  Your write medium must sustain a rate of %fMB/s.\n"
                           "  Dropped samples will not be written to the file.\n"
                           "  Please modify this example for your purposes.\n"
                           "  This message will not appear again.\n")
                           % (usrp->get_rx_rate() * sizeof(samp_type) / 1e6);
            }
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error("Receiver error " + md.strerror());
        }

        num_total_samps += num_rx_samps;

        if (save_rx){
            for (size_t i = 0; i < outfiles.size(); i++) {
                outfiles[i]->write(
                    (const char*)buff_ptrs[i], num_rx_samps * sizeof(samp_type));
            }
        }

    }

    // Shut down receiver
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    rx_stream->issue_stream_cmd(stream_cmd);

    // Close files
    if (save_rx){
        for (size_t i = 0; i < outfiles.size(); i++) {
            outfiles[i]->close();
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------
// Ber testing function
//--------------------------------------------------------------------------------------------------------------------------------------------
struct BerResult {
    double ber;
    int num_bits;
    int num_errs;
    double rss_dbm; //received signal strength in dbm
};

BerResult BerTest(uhd::usrp::multi_usrp::sptr src_tx_usrp, uhd::usrp::multi_usrp::sptr dest_tx_usrp, double EsN0_db, int const target_errs, const int max_num_bits, bool is_fixed_length) {
    //reset device
    mmio::InitBBCore(src_tx_usrp);
    mmio::InitBBCore(dest_tx_usrp);

    //noise estimation-----------------------------------------------------------------------------------------------------------------------
    std::cout << "Running noise estimation..." << std::endl;
    double var = estim::P2PEstimChipNoise(src_tx_usrp, dest_tx_usrp, std::pow(2,14), "../../data/fwd_p2p_noise_chips.dat"); //../../data/fwd_p2p_noise_samps.dat
    std::cout << "Estimated var= " << var << std::endl;
    
    // Feedback estimation ------------------------------------------------------------------------------------------------------------------
    std::cout << "Running fb estimation..." << std::endl;
    std::complex<double> h_hat_fb;
    int D_test = 0;

    while (true) {
        auto ch_params_fb = estim::P2PChEstim(src_tx_usrp, dest_tx_usrp, D_test, std::pow(2,15), false, 0x0, false, "../../data/fb_p2p_prmbl_samps.dat"); //"../../data/fb_p2p_prmbl_samps.dat"
        int D_hat_fb = ch_params_fb.D_hat;
        h_hat_fb = ch_params_fb.h_hat;
    
        std::cout << std::dec << "D_test= " << D_test << ", ";
        std::cout << "D_hat_fb= " << D_hat_fb << ", ";
        std::cout << "h_hat_fb : abs= " << std::abs(h_hat_fb) << " arg= " << std::arg(h_hat_fb) << std::endl;

        if(D_hat_fb > 0 && D_hat_fb < 500){
            break;
        }
    }

    // Timing+flatfading estimation---------------------------------------------------------------------------------------------------------------------------
    //wired loopback delay with 8inch sma cable + attenuator is 119
    std::cout << "Running fwd estimation..." << std::endl;
    //set sync lock estim and locked periods
    uint32_t sync_start_periods = (0xFFFF << 16) + 0x001F;
    mmio::WrMmio(src_tx_usrp, mmio::kSyncStartPeriodAddr,sync_start_periods);
    mmio::WrMmio(dest_tx_usrp, mmio::kSyncStartPeriodAddr,sync_start_periods);

    int D_hat_fwd;
    std::complex<double> h_hat_fwd;

    while (true) {
        auto ch_params = estim::P2PChEstim(src_tx_usrp, dest_tx_usrp, D_test, std::pow(2,15), true, 0x1, false, "../../data/fwd_p2p_prmbl_samps0.dat"); //std::string("../../data/fwd_p2p_prmbl_samps")+std::to_string(j)+".dat"
        D_hat_fwd = ch_params.D_hat;
        h_hat_fwd = ch_params.h_hat;
        
        if(D_hat_fwd > 0 && D_hat_fwd < 500) {
            break;
        }
    }

    double EsN0 = estim::CalcChipEsN0(h_hat_fwd, var);
    mmio::ClearAddrBuffer(dest_tx_usrp);

    std::cout << std::dec << "D_test= " << D_test << ", ";
    std::cout << "D_hat_fwd= " << D_hat_fwd << ", ";
    std::cout << "EsN0= " << EsN0 << ", ";
    std::cout << "h_hat_fwd : abs= " << std::abs(h_hat_fwd) << " arg= " << std::arg(h_hat_fwd) << std::endl;

    //Gain Control--------------------------------------------------------------------------------------------------------------------------
    //Set operating EsN0
    double target_EsN0 = EsN0_db; //in dB
    std::cout << "Target Es_N0 = " << target_EsN0 << std::endl;
    double target_gain = target_EsN0-EsN0; //change in EsN0 needed to achieve target
    double target_rx_gain = -(20*std::log10(std::abs(h_hat_fwd)) + target_gain); //rx_gain needed to bring signal amplitude to 1
    std::cout << "target_rx_gain (db)= " << target_rx_gain << std::endl;

    double tx_gain_base = src_tx_usrp->get_tx_gain(0); //base tx gain used for smaple capture
    //assume tx amp is max
    //assume rx gain is 0
    
    double lin_digital_gain = std::pow(10,(target_gain)/20);
    uint16_t tx_amp = static_cast<uint16_t>(std::round(lin_digital_gain*(std::pow(2,15)-1)));
    std::cout << std::hex << "tx_amp:" << tx_amp << std::endl;
    mmio::WrMmio(src_tx_usrp,mmio::kSrcTxAmpAddr,tx_amp);
    std::complex<double> h = h_hat_fwd*std::complex<double>(lin_digital_gain,0);
    std::cout << "Current signal level: " <<  abs(h) << std::endl;
    double rss_dbW = 20*log10(std::abs(h) * std::pow(2,4) * std::pow(2,-13)) - 41.81 - 10*log10(50); //50 ohm resistor at end
    std::cout << "Current rss (dbW) is: " << rss_dbW << "\n";
    
    //Set number of bits shifted
    if (abs(h) < 1){        
        double dest_num_bit_shift = std::floor(-std::log2(std::abs(h_hat_fwd)*lin_digital_gain));
        std::cout << std::dec << "Bit shift: " << dest_num_bit_shift << std::endl;

        mmio::WrMmio(dest_tx_usrp, mmio::kDestNumBitShift, dest_num_bit_shift); //shift dest rx by 3 to the left (multiply by 8)
        std::cout << std::dec << "dest_num_bit_shift set to: " << static_cast<unsigned int>(dest_num_bit_shift) << std::endl;
        h = h * std::pow(2,dest_num_bit_shift);
    }
    std::cout << "Current signal level: " <<  abs(h) << std::endl;
    

    //Test setup------------------------------------------------------------------
    std::cout << "Performing compensation..." << std::endl;
    estim::P2PCompensateDelays(src_tx_usrp, dest_tx_usrp, D_hat_fwd);

    // std::complex<double> h_comp = h_hat_fwd/std::abs(h_hat_fwd); 
    estim::PhaseEq(dest_tx_usrp, h); //h_comp

    estim::SetSrcThreshold(src_tx_usrp, h_hat_fb);

    // mmio::RdMmio(src_tx_usrp, mmio::kSrcDelayAddr, true);
    // mmio::RdMmio(dest_tx_usrp, mmio::kDestDelayAddr, true);

    // std::cout << "Source read:" << std::endl;    
    // mmio::ReadBBCore(src_tx_usrp);
    // std::cout << "Dest read:" << std::endl;
    // mmio::ReadBBCore(dest_tx_usrp);

    //Run test------------------------------------------------------------------------------------
    std::cout << "Running BER test..." << std::endl;

    bool fixed_length = is_fixed_length;
    uint8_t fix_len_mode_bits;
    if(fixed_length) {
        fix_len_mode_bits = 0b11;
    } else {
        fix_len_mode_bits = 0b00;
    }
    
    std::uint32_t mode_bits{0b11};

    double n_errors = 0; 

    const int kMaxIter = std::ceil(static_cast<double>(max_num_bits)/mmio::kPktLen);
    const int kTargetErr = target_errs;

    //Generate input bits
    std::random_device rd;
    // Create a Mersenne Twister PRNG engine
    std::mt19937 mt(rd());
    // Define a distribution for generating uint32_t values
    std::uniform_int_distribution<uint32_t> dist;
    
    int n_iters = kMaxIter;

    // Generate a random pkt
    const int Num16BitSlices = mmio::kPktLen/32;
    uint32_t input_pkt[Num16BitSlices] = {0};
    uint32_t output_pkt[Num16BitSlices] = {0};

    for(int iter = 1; iter <= kMaxIter; iter++) {
        // Generate a random uint32_t
        for(int i = 0; i < Num16BitSlices; i++)
        {
            uint32_t randomValue = dist(mt);
            //std::cout << "Random uint32_t: " << std::hex << std::setw(4) << std::setfill('0') << randomValue << std::endl;
            input_pkt[i] = randomValue;
            mmio::WrMmio(src_tx_usrp, mmio::kInPktAddr+i, input_pkt[i]);
        }

        mmio::P2PStartTxRx(src_tx_usrp, dest_tx_usrp, mode_bits, estim::kFwdGpioStartSelBits,fix_len_mode_bits,0x1,true);

        //wait for the next start cycle
        int current_ctr = mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF;
        // std::cout << "Current lock idx: " << current_ctr << std::endl;
        while(true){
            mmio::ClearAddrBuffer(dest_tx_usrp);
            if((mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF) != current_ctr) {
                break;
            }
        }
        current_ctr++;
        
        while(true) {
            //Run and check received pkt    
            mmio::ClearAddrBuffer(dest_tx_usrp);
            bool pkt_valid = mmio::RdMmio(dest_tx_usrp, mmio::kBbStatusAddr) & 0x2; //around 10 ms
            if(pkt_valid)
                break;
        }

        // read results ---------------------------------------------
        for(int i = 0; i*32 < mmio::kPktLen; i++) {
            output_pkt[i] = mmio::RdMmio(dest_tx_usrp, mmio::kOutPktAddr+i);
            //std::cout << std::hex << input_pkt[i] << std::endl;

            uint32_t xor_result = output_pkt[i] ^ input_pkt[i];
            while (xor_result > 0) {
                n_errors += xor_result & 1;
                xor_result >>= 1;
            }
            // std::cout << std::dec << "Pkt: " << iter <<std::endl;
            // std::cout << std::dec << "Bit slice: " << i << " Num errors: "<< n_errors <<std::endl;
            // std::cout << std::hex << "Input:  " << input_pkt[i] << std::endl;
            // std::cout << std::hex << "Output: " << output_pkt[i] << std::endl << std::endl;
        }

        //make sure that sample read wasnt corrupted by the next run
        int post_read_ctr = mmio::RdMmio(dest_tx_usrp, mmio::kSyncCtrAddr) & 0xFFFF;
        if(current_ctr != post_read_ctr) {
            std::cout << "Error: Sync Lock Ctr advanced before samples were done reading! Increase start period or decrease number of samples captured. \n" 
            << "pre  read ctr: " << (current_ctr) << std::endl
            << "post read ctr: " << post_read_ctr << std::endl;
        }

        if(iter % 100 == 0) {
            std::cout << std::dec << "Num bits: " << iter*mmio::kPktLen << ", num errors: " << n_errors << std::endl;
        }
        n_iters = iter;
        if(n_errors > kTargetErr){
            break;
        }
    }

    BerResult ber_result;
    ber_result.num_bits = n_iters*mmio::kPktLen;
    ber_result.num_errs = n_errors;
    ber_result.ber = static_cast<double>(ber_result.num_errs)/static_cast<double>(ber_result.num_bits);
    ber_result.rss_dbm = rss_dbW + 30;
    
    std::cout << "Test EsN0_db = " << EsN0_db << " Fixed length = " << is_fixed_length << std::endl;
    std::cout << std::dec << "Reached " << ber_result.num_errs  << " errors in " << ber_result.num_bits << " bits" << std::endl;
    std::cout << "ber = " << ber_result.ber << std::endl;

    return ber_result;  
};

// // Template function to generate MATLAB array creation code with custom vector name
// template <typename T>
// std::string generateMatlabArray(const std::vector<T>& array, const std::string& vectorName) {
//     static_assert(std::is_arithmetic<T>::value, "Template argument must be numeric");
    
//     std::ostringstream matlabCode;
//     matlabCode << vectorName << " = [";
    
//     for (size_t i = 0; i < array.size(); ++i) {
//         matlabCode << array[i];
//         if (i < array.size()-1) {
//             matlabCode << ", ";
//         }
//     }
    
//     matlabCode << "];\n";
    
//     return matlabCode.str();
// }

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // transmit variables to be set by po
    std::string tx_args, wave_type, tx_ant, tx_subdev, ref, otw, tx_channels;
    double tx_rate, tx_freq, tx_gain, wave_freq, tx_bw;
    float ampl;

    // receive variables to be set by po
    std::string rx_args, file, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, spb, save_file;
    double rx_rate, rx_freq, rx_gain, rx_bw;
    double settling;

    //WW - optional user defined arguments
    uint32_t input_reg, output_reg;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        //usrp selection
        ("tx-args", po::value<std::string>(&tx_args)->default_value("type=x300,addr=192.168.110.2"), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value("type=x300,addr=192.168.110.2"), "uhd receive device address args")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        
        //streaming to file
        ("file", po::value<std::string>(&file)->default_value("stream_samps.dat"), "name of the file to write binary samples to")
        ("save-file", po::value<size_t>(&save_file)->default_value(0), "save file option")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type in file: double, float, or short")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer, 0 for default")
        ("tx-rate", po::value<double>(&tx_rate)->default_value(6.25e6), "rate of transmit outgoing samples") //Data rate of host data
        ("rx-rate", po::value<double>(&rx_rate)->default_value(6.25e6), "rate of receive incoming samples") //Data rate of streaming
        
        //user specified arguments
        ("input reg", po::value<uint32_t>(&input_reg)->default_value(0), "input reg")
        ("output reg", po::value<uint32_t>(&output_reg)->default_value(0), "output reg")

        //afe params
        ("tx-freq", po::value<double>(&tx_freq)->default_value(.915e9), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(.915e9), "receive RF center frequency in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(0), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(0), "gain for the receive RF chain")
        ("tx-bw", po::value<double>(&tx_bw)->default_value(160e6), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw)->default_value(160e6), "analog receive filter bandwidth in Hz")
        
        //afe selection
        ("tx-ant", po::value<std::string>(&tx_ant)->default_value("TX/RX"), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant)->default_value("RX2"), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev)->default_value("A:0"), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev)->default_value("A:0"), "receive subdevice specification")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
        
        //waveform stuff (WW not used)
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("SINE"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(125000), "waveform frequency in Hz")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "UHD TXRX Loopback to File " << desc << std::endl;
        return ~0;
    }

    //USRP initialization------------------------------------------------------------------------------------------------------------------------
    std::string src_args = "type=x300,addr=192.168.110.2"; //top
    std::string dest_args = "type=x300,addr=192.168.10.2"; //bottom
    ref = "external"; //octoclock
    double fwd_freq = 2.2e9; //5.80e9;
    double fb_freq = .915e9; //.915e9;
    double src_tx_gain = 0;
    double dest_tx_gain = 15;

    double src_rx_gain = 0;
    double dest_rx_gain = 0;

    double src_tx_freq = fwd_freq;
    double dest_rx_freq = fwd_freq;
    double src_rx_freq = fb_freq;
    double dest_tx_freq = fb_freq;


    //Src Config-----------------------------------------------------------------------------------------------------------------
    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the src transmit usrp device with: %s...") % src_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr src_tx_usrp = uhd::usrp::multi_usrp::make(src_args);
    std::cout << std::endl;
    std::cout << boost::format("Creating the src receive usrp device with: %s...") % src_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr src_rx_usrp = uhd::usrp::multi_usrp::make(src_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev"))
        src_tx_usrp->set_tx_subdev_spec(tx_subdev);
    if (vm.count("rx-subdev"))
        src_rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> src_tx_channel_strings;
    std::vector<size_t> src_tx_channel_nums;
    boost::split(src_tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < src_tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(src_tx_channel_strings[ch]);
        if (chan >= src_tx_usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            src_tx_channel_nums.push_back(std::stoi(src_tx_channel_strings[ch]));
    }
    std::vector<std::string> src_rx_channel_strings;
    std::vector<size_t> src_rx_channel_nums;
    boost::split(src_rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < src_rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(src_rx_channel_strings[ch]);
        if (chan >= src_rx_usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            src_rx_channel_nums.push_back(std::stoi(src_rx_channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        src_tx_usrp->set_clock_source(ref);
        src_rx_usrp->set_clock_source(ref);
    }

    std::cout << "Using src TX Device: " << src_tx_usrp->get_pp_string() << std::endl;
    std::cout << "Using src RX Device: " << src_rx_usrp->get_pp_string() << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    src_tx_usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (src_tx_usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    src_rx_usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (src_rx_usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    for (size_t ch = 0; ch < src_tx_channel_nums.size(); ch++) {
        size_t src_channel = src_tx_channel_nums[ch];
        if (src_tx_channel_nums.size() > 1) {
            std::cout << "Configuring TX Channel " << src_channel << std::endl;
        }
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (src_tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tx_tune_request(src_tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        src_tx_usrp->set_tx_freq(tx_tune_request, src_channel);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (src_tx_usrp->get_tx_freq(src_channel) / 1e6)
                  << std::endl
                  << std::endl;

        //std::cout << tx_usrp->get_rx_gain_range(channel).step() << std::endl;
        // set the rf gain, ubx range: 0-31.5dB
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % src_tx_gain
                      << std::endl;
            src_tx_usrp->set_tx_gain(src_tx_gain, src_channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % src_tx_usrp->get_tx_gain(src_channel)
                      << std::endl
                      << std::endl;
        }


        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            src_tx_usrp->set_tx_bandwidth(tx_bw, src_channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % src_tx_usrp->get_tx_bandwidth(src_channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            src_tx_usrp->set_tx_antenna(tx_ant, src_channel);
    }

    for (size_t ch = 0; ch < src_rx_channel_nums.size(); ch++) {
        size_t src_channel = src_rx_channel_nums[ch];
        if (src_rx_channel_nums.size() > 1) {
            std::cout << "Configuring RX Channel " << src_channel << std::endl;
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (src_rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(src_rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        src_rx_usrp->set_rx_freq(rx_tune_request, src_channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (src_rx_usrp->get_rx_freq(src_channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain ubx range: 0-31.5dB
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % src_rx_gain
                      << std::endl;
            src_rx_usrp->set_rx_gain(src_rx_gain, src_channel);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % src_rx_usrp->get_rx_gain(src_channel)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            src_rx_usrp->set_rx_bandwidth(rx_bw, src_channel);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (src_rx_usrp->get_rx_bandwidth(src_channel) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            src_rx_usrp->set_rx_antenna(rx_ant, src_channel);
    }

    // //print options
    // std::vector<std::string> gpio_banks;
    // gpio_banks = src_tx_usrp->get_gpio_banks(0);
    // for(const auto& bank : gpio_banks) {
    //     std::cout << bank << std::endl;
    // }
    
    // for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST") {
        wave_freq = src_tx_usrp->get_tx_rate() / 2;
    }

    // error when the waveform is not possible to generate
    if (std::abs(wave_freq) > src_tx_usrp->get_tx_rate() / 2) {
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (src_tx_usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
        throw std::runtime_error("wave freq too small for table");
    }

    // pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t src_step = std::lround(wave_freq / src_tx_usrp->get_tx_rate() * wave_table_len);
    size_t src_index      = 0;

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels             = src_tx_channel_nums;
    uhd::tx_streamer::sptr src_tx_stream = src_tx_usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0)
        spb = src_tx_stream->get_max_num_samps() * 10;
    std::vector<std::complex<float>> src_buff(spb); //dedicated buffer for both transmits
    int src_num_channels = src_tx_channel_nums.size();

    // setup the metadata flags
    uhd::tx_metadata_t src_md;
    src_md.start_of_burst = true;
    src_md.end_of_burst   = false;
    src_md.has_time_spec  = true;
    src_md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers

    // Check Ref and LO Lock detect
    std::vector<std::string> src_tx_sensor_names, src_rx_sensor_names;
    src_tx_sensor_names = src_tx_usrp->get_tx_sensor_names(0);
    if (std::find(src_tx_sensor_names.begin(), src_tx_sensor_names.end(), "lo_locked")
        != src_tx_sensor_names.end()) {
        uhd::sensor_value_t src_lo_locked = src_tx_usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % src_lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_lo_locked.to_bool());
    }
    src_rx_sensor_names = src_rx_usrp->get_rx_sensor_names(0);
    if (std::find(src_rx_sensor_names.begin(), src_rx_sensor_names.end(), "lo_locked")
        != src_rx_sensor_names.end()) {
        uhd::sensor_value_t src_lo_locked = src_rx_usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % src_lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_lo_locked.to_bool());
    }

    src_tx_sensor_names = src_tx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(src_tx_sensor_names.begin(), src_tx_sensor_names.end(), "mimo_locked")
                != src_tx_sensor_names.end())) {
        uhd::sensor_value_t src_mimo_locked = src_tx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % src_mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(src_tx_sensor_names.begin(), src_tx_sensor_names.end(), "ref_locked")
                != src_tx_sensor_names.end())) {
        uhd::sensor_value_t src_ref_locked = src_tx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % src_ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_ref_locked.to_bool());
    }

    src_rx_sensor_names = src_rx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(src_rx_sensor_names.begin(), src_rx_sensor_names.end(), "mimo_locked")
                != src_rx_sensor_names.end())) {
        uhd::sensor_value_t src_mimo_locked = src_rx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % src_mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(src_rx_sensor_names.begin(), src_rx_sensor_names.end(), "ref_locked")
                != src_rx_sensor_names.end())) {
        uhd::sensor_value_t src_ref_locked = src_rx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % src_ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(src_ref_locked.to_bool());
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }
    //For early termination use Ctrl + Z

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    src_tx_usrp->set_time_now(uhd::time_spec_t(0.0));

    src_tx_usrp->set_rx_dc_offset(true);
        
    //Start tx and streaming
    // start transmit worker thread
    std::thread src_transmit_thread([&]() {
        transmit_worker(src_buff, wave_table, src_tx_stream, src_md, src_step, src_index, src_num_channels); //this sets tx_streamer which gates tx
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active

    // recv to file - supposedly sets registers on adc but I cant find anything about that. 
    // However, given how transmit_worker sets the tx settings (tx_running), its very possible that rx settings need to be set for proper operation
    // Ordinary operation of recv_to_file will lock out the rest of the c++ code, so try putting it in a thread so that it can execute indefinitely just like transmit_worker
    // This will block streaming though. If you want to record samples you that will have to modify recv_to_file to write to file for only part of the time recv to file is active.
    //  Or separately call this after a run is complete to capture strobed data...
    std::thread src_recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            src_rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, src_rx_channel_nums, 0); //save_rx = 0 so that we dont create a huge file
    });

    //Dest config-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the dest transmit usrp device with: %s...") % dest_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr dest_tx_usrp = uhd::usrp::multi_usrp::make(dest_args);
    std::cout << std::endl;
    std::cout << boost::format("Creating the dest receive usrp device with: %s...") % dest_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr dest_rx_usrp = uhd::usrp::multi_usrp::make(dest_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev"))
        dest_tx_usrp->set_tx_subdev_spec(tx_subdev);
    if (vm.count("rx-subdev"))
        dest_rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> dest_tx_channel_strings;
    std::vector<size_t> dest_tx_channel_nums;
    boost::split(dest_tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < dest_tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(dest_tx_channel_strings[ch]);
        if (chan >= dest_tx_usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            dest_tx_channel_nums.push_back(std::stoi(dest_tx_channel_strings[ch]));
    }
    std::vector<std::string> dest_rx_channel_strings;
    std::vector<size_t> dest_rx_channel_nums;
    boost::split(dest_rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < dest_rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(dest_rx_channel_strings[ch]);
        if (chan >= dest_rx_usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            dest_rx_channel_nums.push_back(std::stoi(dest_rx_channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        dest_tx_usrp->set_clock_source(ref);
        dest_rx_usrp->set_clock_source(ref);
    }

    std::cout << "Using dest TX Device: " << dest_tx_usrp->get_pp_string() << std::endl;
    std::cout << "Using dest RX Device: " << dest_rx_usrp->get_pp_string() << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    dest_tx_usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (dest_tx_usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    dest_rx_usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (dest_rx_usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    for (size_t ch = 0; ch < dest_tx_channel_nums.size(); ch++) {
        size_t channel = dest_tx_channel_nums[ch];
        if (dest_tx_channel_nums.size() > 1) {
            std::cout << "Configuring TX Channel " << channel << std::endl;
        }
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (dest_tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tx_tune_request(dest_tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        dest_tx_usrp->set_tx_freq(tx_tune_request, channel);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (dest_tx_usrp->get_tx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        //std::cout << tx_usrp->get_rx_gain_range(channel).step() << std::endl;
        // set the rf gain, ubx range: 0-31.5dB
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % dest_tx_gain
                      << std::endl;
            dest_tx_usrp->set_tx_gain(dest_tx_gain, channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % dest_tx_usrp->get_tx_gain(channel)
                      << std::endl
                      << std::endl;
        }


        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            dest_tx_usrp->set_tx_bandwidth(tx_bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % dest_tx_usrp->get_tx_bandwidth(channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            dest_tx_usrp->set_tx_antenna(tx_ant, channel);
    }

    for (size_t ch = 0; ch < dest_rx_channel_nums.size(); ch++) {
        size_t channel = dest_rx_channel_nums[ch];
        if (dest_rx_channel_nums.size() > 1) {
            std::cout << "Configuring RX Channel " << channel << std::endl;
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (dest_rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(dest_rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        dest_rx_usrp->set_rx_freq(rx_tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (dest_rx_usrp->get_rx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain ubx range: 0-31.5dB
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % dest_rx_gain
                      << std::endl;
            dest_rx_usrp->set_rx_gain(dest_rx_gain, channel);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % dest_rx_usrp->get_rx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            dest_rx_usrp->set_rx_bandwidth(rx_bw, channel);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (dest_rx_usrp->get_rx_bandwidth(channel) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            dest_rx_usrp->set_rx_antenna(rx_ant, channel);
    }

    // //print options
    // std::vector<std::string> gpio_banks;
    // gpio_banks = dest_tx_usrp->get_gpio_banks(0);
    // for(const auto& bank : gpio_banks) {
    //     std::cout << bank << std::endl;
    // }
    
    // for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST") {
        wave_freq = dest_tx_usrp->get_tx_rate() / 2;
    }

    // error when the waveform is not possible to generate
    if (std::abs(wave_freq) > dest_tx_usrp->get_tx_rate() / 2) {
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (dest_tx_usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
        throw std::runtime_error("wave freq too small for table");
    }

    // pre-compute the waveform values
    const wave_table_class dest_wave_table(wave_type, ampl);
    const size_t dest_step = std::lround(wave_freq / dest_tx_usrp->get_tx_rate() * wave_table_len);
    size_t dest_index      = 0;

    // create a transmit streamer
    // linearly map channels (dest_index0 = channel0, dest_index1 = channel1, ...)
    uhd::stream_args_t dest_stream_args("fc32", otw);
    dest_stream_args.channels             = dest_tx_channel_nums;
    uhd::tx_streamer::sptr dest_tx_stream = dest_tx_usrp->get_tx_stream(dest_stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0)
        spb = dest_tx_stream->get_max_num_samps() * 10;
    std::vector<std::complex<float>> dest_buff(spb); //dedicated buffer for both transmits
    int dest_num_channels = dest_tx_channel_nums.size();

    // setup the metadata flags
    uhd::tx_metadata_t dest_md;
    dest_md.start_of_burst = true;
    dest_md.end_of_burst   = false;
    dest_md.has_time_spec  = true;
    dest_md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers

    // Check Ref and LO Lock detect
    std::vector<std::string> dest_tx_sensor_names, dest_rx_sensor_names;
    dest_tx_sensor_names = dest_tx_usrp->get_tx_sensor_names(0);
    if (std::find(dest_tx_sensor_names.begin(), dest_tx_sensor_names.end(), "lo_locked")
        != dest_tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = dest_tx_usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    dest_rx_sensor_names = dest_rx_usrp->get_rx_sensor_names(0);
    if (std::find(dest_rx_sensor_names.begin(), dest_rx_sensor_names.end(), "lo_locked")
        != dest_rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = dest_rx_usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    dest_tx_sensor_names = dest_tx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(dest_tx_sensor_names.begin(), dest_tx_sensor_names.end(), "mimo_locked")
                != dest_tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = dest_tx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(dest_tx_sensor_names.begin(), dest_tx_sensor_names.end(), "ref_locked")
                != dest_tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = dest_tx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    dest_rx_sensor_names = dest_rx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(dest_rx_sensor_names.begin(), dest_rx_sensor_names.end(), "mimo_locked")
                != dest_rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = dest_rx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(dest_rx_sensor_names.begin(), dest_rx_sensor_names.end(), "ref_locked")
                != dest_rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = dest_rx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }
    //For early termination use Ctrl + Z

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    dest_tx_usrp->set_time_now(uhd::time_spec_t(0.0));

    dest_tx_usrp->set_rx_dc_offset(true);
        
    //Start tx and streaming
    // start transmit worker thread
    std::thread dest_transmit_thread([&]() {
        transmit_worker(dest_buff, dest_wave_table, dest_tx_stream, dest_md, dest_step, dest_index, dest_num_channels); //this sets tx_streamer which gates tx
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active

    // recv to file - supposedly sets registers on adc but I cant find anything about that. 
    // However, given how transmit_worker sets the tx settings (tx_running), its very possible that rx settings need to be set for proper operation
    // Ordinary operation of recv_to_file will lock out the rest of the c++ code, so try putting it in a thread so that it can execute indefinitely just like transmit_worker
    // This will block streaming though. If you want to record samples you that will have to modify recv_to_file to write to file for only part of the time recv to file is active.
    //  Or separately call this after a run is complete to capture strobed data...
    std::thread dest_recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            dest_rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, dest_rx_channel_nums, 0); //save_rx = 0 so that we dont create a huge file
    });

    //--------------------------------------------------------------------------------------------------------------------------
    //WW - OSLA-BPSK Operation
    //--------------------------------------------------------------------------------------------------------------------------

    //Preload some default threshold and angle settings
    // mmio::InitBBCore(src_tx_usrp);
    // mmio::InitBBCore(dest_tx_usrp);

    std::vector<double> EsN0_dbs = {0,1,2,3,4,5,6};//{4,5,6,7};//{0,1,2,3,4,5,6,7};
    std::vector<double> bers(EsN0_dbs.size(), 0.0);
    std::vector<int> num_errs(EsN0_dbs.size(), 0);
    std::vector<int> num_bits(EsN0_dbs.size(), 0);
    std::vector<double> rss_dbms(EsN0_dbs.size(), 0.0);

    const int kTargetErrs = 500;
    const int kMaxBits = 1e6;

    bool is_fixed_length = false;

    for(int i = 0; i<EsN0_dbs.size(); i++) {
        std::cout << "Running BER test for EsN0_db = " << EsN0_dbs[i] << std::endl;
        BerResult ber_result = BerTest(src_tx_usrp, dest_tx_usrp, EsN0_dbs[i], kTargetErrs, kMaxBits, is_fixed_length);
        bers[i] = ber_result.ber;
        num_bits[i] = ber_result.num_bits;
        num_errs[i] = ber_result.num_errs;
        rss_dbms[i] = ber_result.rss_dbm;
    }

    std::cout << estim::generateMatlabArray(EsN0_dbs, "EsN0_dbs");
    std::cout << estim::generateMatlabArray(bers,"bers");
    std::cout << estim::generateMatlabArray(num_bits,"num_bits");
    std::cout << estim::generateMatlabArray(num_errs,"num_errs");
    std::cout << estim::generateMatlabArray(rss_dbms,"rss_dbms");

    //////////////////////////////////////////////////////////////////////////////////////////////////

    // clean up transmit worker
    stop_signal_called = true;
    src_transmit_thread.join();
    src_recv_thread.join();
    dest_transmit_thread.join();
    dest_recv_thread.join();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}

