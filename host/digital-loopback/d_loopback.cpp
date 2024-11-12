//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
//WW-This file is a modified version of uhd/host/examples/txrx_loopback_to_file.cpp. It essentially sets up + activates the radios and writes & sends samples to file

#include "wavetable.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <csignal>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>

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
 * A function to be used as a boost::thread_group thread for transmitting
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
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

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
    double timeout =
        settling_time + 0.1f; // expected settling time + padding for first recv

    // setup streaming
    uhd::stream_cmd_t stream_cmd((num_requested_samples == 0)
                                     ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS
                                     : uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = uhd::time_spec_t(settling_time);
    rx_stream->issue_stream_cmd(stream_cmd);

    while (not stop_signal_called
           and (num_requested_samples > num_total_samps or num_requested_samples == 0)) {
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samps_per_buff, md, timeout);
        timeout             = 0.1f; // small timeout for subsequent recv

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
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
            throw std::runtime_error(
                str(boost::format("Receiver error %s") % md.strerror()));
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

//-------------------------------------------------------------------
//WW Functions
//--------------------------------------------------------------------

/*
Write a command to gpio-in and then reads the contents of gpio-out and prints it to the console 
*/
uint32_t rd_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, const int ms_delay = 1)
{
   if(cmd >> 31) //check to make sure cmd is a read command
        std::cout << "WARNING: Write command used where read command was expected. cmd: " << cmd << std::endl;

   tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
   std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); 

   uint32_t output_reg;
   output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK"); 
   std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay));

   //std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl; 
   return output_reg;
}


void wr_mem_cmd(uhd::usrp::multi_usrp::sptr tx_usrp, const uint32_t cmd, const int ms_delay = 1)
{
    if(cmd >> 31 == 0) //check to make sure cmd is a wr command
        std::cout << "WARNING: Read command use where write command was expected. cmd: " << cmd << std::endl;

    tx_usrp->set_gpio_attr("FP0", "OUT", cmd); 
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); 
}


//--------------------------------------------------------------------


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
    uint32_t input_reg, output_reg;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("tx-args", po::value<std::string>(&tx_args)->default_value("type=x300"), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&rx_args)->default_value("type=x300"), "uhd receive device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to write binary samples to")
        ("save-file", po::value<size_t>(&save_file)->default_value(0), "save file option")
        ("input reg", po::value<uint32_t>(&input_reg)->default_value(0), "input reg")
        ("output reg", po::value<uint32_t>(&output_reg)->default_value(0), "output reg")

        ("type", po::value<std::string>(&type)->default_value("short"), "sample type in file: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("settling", po::value<double>(&settling)->default_value(double(0.2)), "settling time (seconds) before receiving")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer, 0 for default")
        ("tx-rate", po::value<double>(&tx_rate)->default_value(6.25e6), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate)->default_value(6.25e6), "rate of receive incoming samples")
        ("tx-freq", po::value<double>(&tx_freq)->default_value(2.4e9), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(2.4e9), "receive RF center frequency in Hz")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of the waveform [0 to 0.7]")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(0), "gain for the transmit RF chain")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(0), "gain for the receive RF chain")
        ("tx-ant", po::value<std::string>(&tx_ant)->default_value("TX/RX"), "transmit antenna selection")
        ("rx-ant", po::value<std::string>(&rx_ant)->default_value("RX2"), "receive antenna selection")
        ("tx-subdev", po::value<std::string>(&tx_subdev)->default_value("A:0"), "transmit subdevice specification")
        ("rx-subdev", po::value<std::string>(&rx_subdev)->default_value("A:0"), "receive subdevice specification")
        ("tx-bw", po::value<double>(&tx_bw)->default_value(160e6), "analog transmit filter bandwidth in Hz")
        ("rx-bw", po::value<double>(&rx_bw)->default_value(160e6), "analog receive filter bandwidth in Hz")
        ("wave-type", po::value<std::string>(&wave_type)->default_value("SINE"), "waveform type (CONST, SQUARE, RAMP, SINE)")
        ("wave-freq", po::value<double>(&wave_freq)->default_value(125000), "waveform frequency in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("tx-channels", po::value<std::string>(&tx_channels)->default_value("0"), "which TX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("rx-channels", po::value<std::string>(&rx_channels)->default_value("0"), "which RX channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx-int-n", "tune USRP TX with integer-N tuning")
        ("rx-int-n", "tune USRP RX with integer-N tuning")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD TXRX Loopback to File %s") % desc << std::endl;
        return ~0;
    }

    // create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the transmit usrp device with: %s...") % tx_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr tx_usrp = uhd::usrp::multi_usrp::make(tx_args);
    std::cout << std::endl;
    std::cout << boost::format("Creating the receive usrp device with: %s...") % rx_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr rx_usrp = uhd::usrp::multi_usrp::make(rx_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev"))
        tx_usrp->set_tx_subdev_spec(tx_subdev);
    if (vm.count("rx-subdev"))
        rx_usrp->set_rx_subdev_spec(rx_subdev);

    // detect which channels to use
    std::vector<std::string> tx_channel_strings;
    std::vector<size_t> tx_channel_nums;
    boost::split(tx_channel_strings, tx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < tx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(tx_channel_strings[ch]);
        if (chan >= tx_usrp->get_tx_num_channels()) {
            throw std::runtime_error("Invalid TX channel(s) specified.");
        } else
            tx_channel_nums.push_back(std::stoi(tx_channel_strings[ch]));
    }
    std::vector<std::string> rx_channel_strings;
    std::vector<size_t> rx_channel_nums;
    boost::split(rx_channel_strings, rx_channels, boost::is_any_of("\"',"));
    for (size_t ch = 0; ch < rx_channel_strings.size(); ch++) {
        size_t chan = std::stoi(rx_channel_strings[ch]);
        if (chan >= rx_usrp->get_rx_num_channels()) {
            throw std::runtime_error("Invalid RX channel(s) specified.");
        } else
            rx_channel_nums.push_back(std::stoi(rx_channel_strings[ch]));
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        tx_usrp->set_clock_source(ref);
        rx_usrp->set_clock_source(ref);
    }

    std::cout << boost::format("Using TX Device: %s") % tx_usrp->get_pp_string()
              << std::endl;
    std::cout << boost::format("Using RX Device: %s") % rx_usrp->get_pp_string()
              << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    tx_usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (tx_usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    rx_usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (rx_usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }

    for (size_t ch = 0; ch < tx_channel_nums.size(); ch++) {
        size_t channel = tx_channel_nums[ch];
        if (tx_channel_nums.size() > 1) {
            std::cout << "Configuring TX Channel " << channel << std::endl;
        }
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (tx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t tx_tune_request(tx_freq);
        if (vm.count("tx-int-n"))
            tx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        tx_usrp->set_tx_freq(tx_tune_request, channel);
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (tx_usrp->get_tx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the rf gain
        if (vm.count("tx-gain")) {
            std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                      << std::endl;
            tx_usrp->set_tx_gain(tx_gain, channel);
            std::cout << boost::format("Actual TX Gain: %f dB...")
                             % tx_usrp->get_tx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            tx_usrp->set_tx_bandwidth(tx_bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % tx_usrp->get_tx_bandwidth(channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            tx_usrp->set_tx_antenna(tx_ant, channel);
    }

    for (size_t ch = 0; ch < rx_channel_nums.size(); ch++) {
        size_t channel = rx_channel_nums[ch];
        if (rx_channel_nums.size() > 1) {
            std::cout << "Configuring RX Channel " << channel << std::endl;
        }

        // set the receive center frequency
        if (not vm.count("rx-freq")) {
            std::cerr << "Please specify the center frequency with --rx-freq"
                      << std::endl;
            return ~0;
        }
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (rx_freq / 1e6)
                  << std::endl;
        uhd::tune_request_t rx_tune_request(rx_freq);
        if (vm.count("rx-int-n"))
            rx_tune_request.args = uhd::device_addr_t("mode_n=integer");
        rx_usrp->set_rx_freq(rx_tune_request, channel);
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (rx_usrp->get_rx_freq(channel) / 1e6)
                  << std::endl
                  << std::endl;

        // set the receive rf gain
        if (vm.count("rx-gain")) {
            std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                      << std::endl;
            rx_usrp->set_rx_gain(rx_gain, channel);
            std::cout << boost::format("Actual RX Gain: %f dB...")
                             % rx_usrp->get_rx_gain(channel)
                      << std::endl
                      << std::endl;
        }

        // set the receive analog frontend filter bandwidth
        if (vm.count("rx-bw")) {
            std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                      << std::endl;
            rx_usrp->set_rx_bandwidth(rx_bw, channel);
            std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                             % (rx_usrp->get_rx_bandwidth(channel) / 1e6)
                      << std::endl
                      << std::endl;
        }

        // set the receive antenna
        if (vm.count("rx-ant"))
            rx_usrp->set_rx_antenna(rx_ant, channel);
    }

    //--------------------------------------------------
    //WW Changes
    //--------------------------------------------------
    constexpr std::uint32_t cmdBits{0xF0000000}; 
    constexpr std::uint32_t addrBits{0x0FFF0000};
    constexpr std::uint32_t dataBits{0x0000FFFF};


    constexpr uint32_t start_cmd = 0x80010001;
    constexpr uint32_t rst_cmd = 0x80010002;

    //Load phases and thresholds
    constexpr int Num_Write_Cmds = 11;
    uint32_t write_cmds[Num_Write_Cmds] = {
        0x80000000,
        0x80010002,
        0x80101DED,
        //0x8020FFFF,
        //0x8020CE94,
        //0x8021FFFF,
        //0x8021E12A,
        0x8050A000,
        0x80510005,
        0x80520453,
        0x80533937,
        0x8060030B,
        0x80613DD1,
        0x8062223C,
        0x8063CA59,
    };

    //Readback input bit, phase, and threshold settings, valid and done, and pkt out
    uint32_t read_cmds[16] = {
        0x00000000, //No OP
        0x00010000, //rst and start
        0x00100000, //src threshold
        0x00200000, //input pkt
        0x00210000, //input pkt
        0x00500000, //dest threshold
        0x00510000, //dest threshold
        0x00520000, //dest ch eq re
        0x00530000, //dest ch eq im
        0x00600000, //fb ch eq re
        0x00610000, //fb ch eq im
        0x00620000, //fw ch eq re
        0x00630000, //fw ch eq im
        
        0x08000000, //valid and done
        0x08100000, //pkt out
        0x08110000  //pkt out
    };

    constexpr std::uint32_t isValid{0x08000001};
    constexpr int ms_delay{1};

    double n_errors{0}; 

    //Generate input bits ----------------------------------------------------
    std::random_device rd;

    // Create a Mersenne Twister PRNG engine
    std::mt19937 mt(rd());

    // Define a distribution for generating uint32_t values
    std::uniform_int_distribution<uint16_t> dist;

    //Threshold and angle settings
    for(int i=0; i<Num_Write_Cmds; i++) {
        tx_usrp->set_gpio_attr("FP0", "OUT", write_cmds[i]);

        std::this_thread::sleep_for(std::chrono::milliseconds(ms_delay)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 
    }


    const int n_iters = 1e4, target_n_errors = 100;
    for (int iter = 0; iter < n_iters; iter++)
    {

        
        //std::cout << "Writing to regs...\n";

        // Generate a random uint32_t
        const int Num16BitSlices = 2;
        uint16_t input_pkt[Num16BitSlices] = {0};
        uint16_t output_pkt[Num16BitSlices] = {0};

        // Generate a random uint32_t
        for(int i = 0; i < Num16BitSlices; i++)
        {
            uint32_t randomValue = dist(mt);
            //std::cout << "Random uint32_t: " << std::hex << std::setw(4) << std::setfill('0') << randomValue << std::endl;

            input_pkt[i] = randomValue;

            uint32_t cmd = 0;
            cmd  = (cmd & ~addrBits) | (0x020+i<<16);
            cmd  = (cmd & ~dataBits) | (randomValue<<0);
            cmd  = (cmd & ~cmdBits) | (1<<31);
        
            wr_mem_cmd(tx_usrp, cmd);
            //std::cout << std::hex << cmd << std::endl;
        }

        

        // ------------------------------------------------------------------------

    
        bool pkt_done = 0;
        
        // Readback ---------------------------------------
        //std::cout << "Readback\n";
        // for(int i=0; i<16; i++) {
        //     rd_mem_cmd(tx_usrp, read_cmds[i]);
        // }
        // ////Debug
        // rd_mem_cmd(tx_usrp, 0x08200000); //db-fb-counter

        // rd_mem_cmd(tx_usrp, 0x08210000); //db-dest-rx-1

        // rd_mem_cmd(tx_usrp, 0x08220000); //db-src-next-pkt-idx

        // rd_mem_cmd(tx_usrp, 0x08300000); //db-dest-pkt-idx

        // --------------------------------------------------

        // start
        wr_mem_cmd(tx_usrp, rst_cmd);
        wr_mem_cmd(tx_usrp, start_cmd);


        // read results ---------------------------------------------
        // //std::cout << "Read results\n";
        // for(int i=0; i<16; i++) {
        //     rd_mem_cmd(tx_usrp, read_cmds[i]);
        // }

        // //debug
        // rd_mem_cmd(tx_usrp, 0x08200000); //db-fb-counter

        // rd_mem_cmd(tx_usrp, 0x08210000); //db-dest-rx-1

        // rd_mem_cmd(tx_usrp, 0x08220000); //db-src-next-pkt-idx

        // rd_mem_cmd(tx_usrp, 0x08300000); //db-dest-pkt-idx



        //Read output bits
        for(int i = 0; i < Num16BitSlices; i++)
        {
            uint32_t cmd = 0;
            cmd  = (cmd & ~addrBits) | (0x810+i<<16);
            cmd  = (cmd & ~dataBits) | (0<<0);
            cmd  = (cmd & ~cmdBits) | (0<<31);
        
            output_pkt[i] = rd_mem_cmd(tx_usrp, cmd) & dataBits;
            uint16_t xor_result = output_pkt[i] ^ input_pkt[i];
            while (xor_result > 0) {
                n_errors += xor_result & 1;
                xor_result >>= 1;
            }

            //std::cout << std::dec << "Bit slice: " << i << std::endl;
            //std::cout << std::hex << "Input:  " << input_pkt[i] << std::endl;
            //std::cout << std::hex << "Output: " << output_pkt[i] << std::endl << std::endl;

        }

        if (iter % 100 == 0) {
            std::cout << "Iters: " << std::dec << iter <<" Num errors: " << std::dec << n_errors << std::endl;
        }
        

        if (n_errors >= target_n_errors)
            break;
    } 

    std::cout << "Test done" << std::endl;
    std::cout << "Num errors: " << std::dec << n_errors << std::endl;



    //Reset 
    /*

    for(int i=0;i<3;i++){

        tx_usrp->set_gpio_attr("FP0", "OUT", rst_cmd); std::this_thread::sleep_for(std::chrono::milliseconds(10000)); 

        tx_usrp->set_gpio_attr("FP0", "OUT", start_cmd); std::this_thread::sleep_for(std::chrono::milliseconds(10)); 

        std::cout << "Reset and run again " << i << std::endl;

        rd_mem_cmd(tx_usrp, 0x08000000); //Check done/valied 

        rd_mem_cmd(tx_usrp, 0x08100000); //Check bits 1-16

        rd_mem_cmd(tx_usrp, 0x08110000); //Check bits 17-32


        //debug
        rd_mem_cmd(tx_usrp, 0x08200000); //db-fb-counter

        rd_mem_cmd(tx_usrp, 0x08210000); //db-dest-rx-1

        rd_mem_cmd(tx_usrp, 0x08220000); //db-src-next-pkt-idx

        rd_mem_cmd(tx_usrp, 0x08300000); //db-dest-pkt-idx
			 
    }
     
    */

    

    // tx_usrp->set_gpio_attr("FP0", "OUT", read_cmds[i]);

    // output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
    // std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // for(int i=1;i<18;i=i+2){
    //     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[i], mask);
        
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
    //     std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl;


    // }

    // std::cout << "Print output\n";

    // for(int i=0;i<3;i++){
    //     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[29], mask);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[2], mask);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	//     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[18], mask);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
	//     std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl;


	//     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[19], mask);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
	//     std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl;

        
	//     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[20], mask);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
	//     std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg << std::endl; 
        
    // }


    // while(!pkt_done)
    // {
	// tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[18], mask);
        
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
	// std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg;
        
	// pkt_done = (output_reg & isValid) == isValid;
 // remember to reset
    // }
    
    // std::cout << "Digital Loopback done. Reading results\n";

    // for(int i=18; i<32; i++) {
    //     tx_usrp->set_gpio_attr("FP0", "CTRL", 0, mask);
    //     tx_usrp->set_gpio_attr("FP0", "OUT", bit_cmds[i], mask);

    //     output_reg = tx_usrp->get_gpio_attr("FP0", "READBACK");
    //     std::cout << std::hex << std::setw(8) << std::setfill('0') << output_reg;
    //     std::cout << std::endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // } 
    // std::cout << "Done printing digital loopback results...";
  
    //////////////////////////////////////////////////////////////////////////////////////////////////
    
    // for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST") {
        wave_freq = tx_usrp->get_tx_rate() / 2;
    }

    // error when the waveform is not possible to generate
    if (std::abs(wave_freq) > tx_usrp->get_tx_rate() / 2) {
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (tx_usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
        throw std::runtime_error("wave freq too small for table");
    }

    // pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t step =
        boost::math::iround(wave_freq / tx_usrp->get_tx_rate() * wave_table_len);
    size_t index = 0;

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels             = tx_channel_nums;
    uhd::tx_streamer::sptr tx_stream = tx_usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0)
        spb = tx_stream->get_max_num_samps() * 10;
    std::vector<std::complex<float>> buff(spb);
    int num_channels = tx_channel_nums.size();

    // setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers

    // Check Ref and LO Lock detect
    std::vector<std::string> tx_sensor_names, rx_sensor_names;
    tx_sensor_names = tx_usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = tx_usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    rx_sensor_names = rx_usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = rx_usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    tx_sensor_names = tx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = tx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = tx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    rx_sensor_names = rx_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = rx_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = rx_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    tx_usrp->set_time_now(uhd::time_spec_t(0.0));

    // start transmit worker thread
    boost::thread_group transmit_thread;
    transmit_thread.create_thread(std::bind(
        &transmit_worker, buff, wave_table, tx_stream, md, step, index, num_channels));

    // recv to file
    bool rx_save = save_file != 0;
    if (type == "double")
        recv_to_file<std::complex<double>>(
            rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    else if (type == "float")
        recv_to_file<std::complex<float>>(
            rx_usrp, "fc32", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    else if (type == "short")
        recv_to_file<std::complex<short>>(
            rx_usrp, "sc16", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    else {
        // clean up transmit worker
        stop_signal_called = true;
        transmit_thread.join_all();
        throw std::runtime_error("Unknown type " + type);
    }

    // clean up transmit worker
    stop_signal_called = true;
    transmit_thread.join_all();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
