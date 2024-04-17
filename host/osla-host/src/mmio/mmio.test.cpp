//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
//WW-This file is a modified version of uhd/host/examples/txrx_loopback_to_file.cpp. It essentially sets up + activates the radios and writes & sends samples to file

#include "../../wavetable.hpp"
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
#include <chrono>

#include "mmio.h"


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
        ("tx-rate", po::value<double>(&tx_rate)->default_value(6.25e6), "rate of transmit outgoing samples")
        ("rx-rate", po::value<double>(&rx_rate)->default_value(6.25e6), "rate of receive incoming samples")
        
        //user specified arguments
        ("input reg", po::value<uint32_t>(&input_reg)->default_value(0), "input reg")
        ("output reg", po::value<uint32_t>(&output_reg)->default_value(0), "output reg")

        //afe params
        ("tx-freq", po::value<double>(&tx_freq)->default_value(2.4e9), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(2.4e9), "receive RF center frequency in Hz")
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

    std::cout << "Using TX Device: " << tx_usrp->get_pp_string() << std::endl;
    std::cout << "Using RX Device: " << rx_usrp->get_pp_string() << std::endl;

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
    const size_t step = std::lround(wave_freq / tx_usrp->get_tx_rate() * wave_table_len);
    size_t index      = 0;

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


    //--------------------------------------------------
    //WW Changes
    //--------------------------------------------------
    
    //Start tx and streaming
    // start transmit worker thread
    std::thread transmit_thread([&]() {
        transmit_worker(buff, wave_table, tx_stream, md, step, index, num_channels); //this sets tx_streamer which gates tx
    });

    // recv to file - supposedly sets registers on adc but I cant find anything about that. 
    // However, given how transmit_worker sets the tx settings (tx_running), its very possible that rx settings need to be set for proper operation
    // Ordinary operation of recv_to_file will lock out the rest of the c++ code, so try putting it in a thread so that it can execute indefinitely just like transmit_worker
    // This will block streaming though. If you want to record samples you that will have to modify recv_to_file to write to file for only part of the time recv to file is active.
    std::thread recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, rx_channel_nums, 0); //save_rx = 0 so that we dont create a huge file
    });

    // //Load phases and thresholds
    // std::vector<uint64_t> write_cmds = {
    //     0x80000000'00000000,
    //     0x80000001'00000001,
    //     0x80000010'00001000,
    //     0x80000011'00000000,
    //     0x80000020'E12ACE94,
    //     0x80000021'E12ACE94,
    //     0x80000030'27100000,
    //     0x80000031'00002000,
    //     0x80000032'00000000,
    //     0x80000033'00000075,
    //     0x80000034'00007FFF
    // };

    // //Readback input bit, phase, and threshold settings, valid and done, and pkt out
    // std::vector<uint32_t> read_cmds = {
    //     0x00000000,
    //     0x00000001,
    //     0x00000010,
    //     0x00000011,
    //     0x00000020,
    //     0x00000021,
    //     0x00000030,
    //     0x00000031,
    //     0x00000032,
    //     0x00000033,
    //     0x00000034,
        
    //     0x00000800,
    //     0x00000810,
    //     0x00000819,
    //     0x02000000,
    //     0x020003FF
    // };

    /*
    //Basic MMIO development - simple read and write
    //write
    std::cout << "\nBasic MMIO test..." << std::endl;
    tx_usrp->set_gpio_attr("FP0A", "OUT", 0x0); 
    tx_usrp->set_gpio_attr("FP0B", "OUT", 0x0); 

    uint64_t test_wr_cmd = 0x80000020'E12ACE94;
    uint32_t test_rd_cmd = 0x00000020;

    tx_usrp->set_gpio_attr("FP0A", "OUT", static_cast<uint32_t>(test_wr_cmd)); 
    tx_usrp->set_gpio_attr("FP0B", "OUT", static_cast<uint32_t>(test_wr_cmd >> 32)); 
    std::this_thread::sleep_for(std::chrono::nanoseconds(5)); //Arguably no delay is necessary per https://stackoverflow.com/questions/18071664/stdthis-threadsleep-for-and-nanoseconds 
    
    std::cout << std::hex << std::setw(16) << std::setfill('0') << "Output after write: " << tx_usrp->get_gpio_attr("FP0B", "READBACK") << std::endl; 
    std::cout << std::hex << std::setw(16) << std::setfill('0') << "Output after write: " << tx_usrp->get_gpio_attr("FP0A", "READBACK") << std::endl; 

    tx_usrp->set_gpio_attr("FP0B", "OUT", test_rd_cmd); 
    std::this_thread::sleep_for(std::chrono::nanoseconds(5)); 
    std::cout << std::hex << std::setw(16) << std::setfill('0') << "Output after read: " <<tx_usrp->get_gpio_attr("FP0A", "READBACK") << std::endl; 
    */

    
    //Full readback test
    //Threshold and angle settings
    std::cout << "\nFull readback test...\n";
    std::cout << "Writing to mmio...\n";
    // for(const auto& cmd : write_cmds) {
    //     wr_mem_cmd(tx_usrp, cmd);
    // }
    mmio::InitBBCore(tx_usrp);
    //Write input pkt
    const int PktLen = 256;
    for(int i = 0; i*32 < PktLen; i++) {
        uint64_t cmd = 0x80000020 + i;

        mmio::wr_mem_cmd(tx_usrp, cmd << 32 | 0xA12ACE94);
        mmio::RdMmio(tx_usrp, 0x00000020+i ,true);
    }

    std::cout << "Readback...\n";
    // for(const auto& cmd : read_cmds) {
    //     rd_mem_cmd(tx_usrp, cmd,true);
    // }
    mmio::ReadBBCore(tx_usrp);

    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
    
    //set digital loopback
    // std::uint32_t mode_bits_shift{0x0 << 2};
    // std::uint32_t rx_ch_sel_bits_shift{0x0 << 4}; 
    // std::uint32_t tx_core_bits_shift{0x0 << 6}; 
    // std::uint32_t gpio_start_sel_bits_shift{0x0 << 8};

    // uint64_t start_cmd = 0x80000001'00000002+mode_bits_shift+rx_ch_sel_bits_shift+tx_core_bits_shift+gpio_start_sel_bits_shift;
    //wr_mem_cmd(tx_usrp, start_cmd);
    
    std::cout << "Start command issued...\n";
    //start_tx(tx_usrp, 0x0, 0x1, 0x2, 0x0); //wired loopback
    mmio::start_tx(tx_usrp, 0b11, 0b0, 0b0, 0b0); //digital loopback

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Need to sleep for at least 500 ms before tx is active

    std::cout << "Reading results...\n";
    mmio::ReadBBCore(tx_usrp);
    
    std::cout << "Done printing digital loopback results...\n";

    //On-chip acquisition test
    std::cout << "On chip acquisition test...\n";
    std::vector<std::complex<double>> cap_samps = mmio::ReadSampleMem(tx_usrp, 1, pow(2,16)-1, "dest_samps.dat");

    int i = 0;
    for(int i = 0; i < 5; i++) {
        std::cout << std::dec << i << ": " << cap_samps[i] << std::endl;
    } //disable printout to look at mmio results

    cap_samps = mmio::ReadSampleMem(tx_usrp, 0, pow(2,16)-1, "src_samps.dat");
    

    // //Write input pkt
    // const int PktLen = 256;
    // for(int i = 0; i*32 < PktLen; i++) {
    //     uint64_t cmd = 0x80000020 + i;

    //     wr_mem_cmd(tx_usrp, cmd << 32 | 0xE12ACE94);
    //     rd_mem_cmd(tx_usrp, 0x00000020+i ,true);
    // }
    

    //pkt write test
    //     std::random_device rd;

    // // Create a Mersenne Twister PRNG engine
    // std::mt19937 mt(rd());

    // // Define a distribution for generating uint32_t values
    // std::uniform_int_distribution<uint32_t> dist;


    // // Generate a random uint32_t
    //     const int Num16BitSlices = PktLen/32;
    //     uint32_t input_pkt[Num16BitSlices] = {0};
    //     uint32_t output_pkt[Num16BitSlices] = {0};

    //     // Generate a random uint32_t
    //     for(int i = 0; i < Num16BitSlices; i++)
    //     {
    //         uint32_t randomValue = dist(mt);
    //         //std::cout << "Random uint32_t: " << std::hex << std::setw(4) << std::setfill('0') << randomValue << std::endl;

    //         input_pkt[i] = randomValue;

    //         WrCmd(tx_usrp, 0x020+i, randomValue);

    //         // uint64_t cmd = 0;
    //         // cmd  = (cmd & ~addrBits) | (static_cast<uint64_t>(0x020+i)<<32);
    //         // cmd  = (cmd & ~dataBits) | (randomValue<<0);
    //         // cmd  = (cmd & ~cmdBits) | (static_cast<uint64_t>(0x1)<<63);
        
    //         // wr_mem_cmd(tx_usrp, cmd);
    //     }
        
// //check write
//         for(int i = 0; i*32 < mmio::kPktLen; i++) {
//             mmio::rd_mem_cmd(tx_usrp, mmio::kPktAddr+i ,true);
//             std::cout << std::hex << input_pkt[i] << std::endl;
//         }

        // //Basic analog loopback test   
    // //Write input pkt
    // const int PktLen = 256;
    // for(int i = 0; i*32 < PktLen; i++) {
    //     uint64_t cmd = 0x80000020 + i;

    //     wr_mem_cmd(tx_usrp, cmd << 32 | 0xE12ACE94);
    //     rd_mem_cmd(tx_usrp, 0x00000020+i ,true);
    // }

    // mode_bits = 0x3;
    // rx_ch_sel_bits = 0x1; 
    // tx_core_bits = 0x2; 
    // gpio_start_sel_bits = 0x0;
    // start_tx(tx_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits);


    // ReadBBCore(tx_usrp);

    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // for(int i = 0; i*32 < PktLen; i++) {
    //     rd_mem_cmd(tx_usrp, 0x00000810+i ,true);
    // }

    /*
    const int NumPrmblSamps_test = 512;
    
    const uint16_t PrmblStartAddr_test = 0xA00;
    

    file = "test1.dat";
    std::ofstream of_file(file, std::ios::binary | std::ios::trunc);

    for(int i = 0; i < NumPrmblSamps_test; i++)
    {
        if (of_file.is_open()) {
            uint32_t cmd = 0;
            cmd  = (cmd & ~addrBits) | (PrmblStartAddr_test+i);
            cmd  = (cmd & ~cmdBits) | (0<<31);
            uint32_t prmbl_samp = static_cast<uint32_t>(rd_mem_cmd(tx_usrp, cmd));

            uint16_t prmbl_samp_I, prmbl_samp_Q;
            prmbl_samp_I = static_cast<uint16_t>(prmbl_samp >> 16); //shouldnt do sign extension
            prmbl_samp_Q = static_cast<uint16_t>(prmbl_samp & 0x0000FFFF);
            // Write the 32 bits to the file
            of_file.write(reinterpret_cast<const char*>(&prmbl_samp_I), sizeof(prmbl_samp_I));
            of_file.write(reinterpret_cast<const char*>(&prmbl_samp_Q), sizeof(prmbl_samp_Q));

            // Write sample to vector
            std::complex<double> samp = {static_cast<double>(prmbl_samp_I),static_cast<double>(prmbl_samp_Q)};
            // Scale this to where we expect the "decimal point" is
            samp *= static_cast<std::complex<double>>(pow(2, -8));
            cap_samps.push_back(samp);
            std::cout << std::dec << i << " " << samp << std::endl;

        } else {
            std::cerr << "Unable to open file for appending." << std::endl;
        }

    }
    
    // Close the file
    of_file.close();

    std::cout << "Samples written to file" << std::endl;
    */


   
/*
    //PN acquisition test
    //Because our window is small, need to sweep multiple time intervals by adjusting source and dest delay. Assumes channel coherence is quite long
    //Multiple tests have confirmed wired loopback delay with 8inch sma cable + attenuator is 119, so its find to just do one interval for now
    int N_sweep_intervals = 0; //50
    std::vector<int> D_hat_sweep, D_test_sweep;
    std::vector<std::complex<double>> h_hat_sweep;
    
    const int DelaySweepInterval = 128;

    for(int interval_idx = -N_sweep_intervals/2; interval_idx <= N_sweep_intervals/2; interval_idx++) {

        int D_test = interval_idx * DelaySweepInterval; //D_test is the delay between src and dest we set. This is the opposite of D_comp's logic
        
        
        uint16_t dest_delay_test, src_delay_test;
        if (D_test < 0) { //estimated D is negative->dest starts first since preamble is early
            dest_delay_test = 0;
            src_delay_test = -D_test;
        }
        else { 
            dest_delay_test = D_test;
            src_delay_test = 0;
        }

        std::cout << std::dec << "D_test= " << D_test << std::endl;
        std::cout << std::dec << "src_delay_test= " << src_delay_test << std::endl;
        std::cout << std::dec << "dest_delay_test= " << dest_delay_test << std::endl;

        //set test delay
        //std::cout << ((0x80110000 & ~dataBits) | src_delay_test) << std::endl;
        //std::cout << ((0x80540000 & ~dataBits) | dest_delay_test) << std::endl;
        wr_mem_cmd(tx_usrp, (0x80110000 & ~dataBits) | src_delay_test);
        wr_mem_cmd(tx_usrp, (0x80540000 & ~dataBits) | dest_delay_test);

        //rd_mem_cmd(tx_usrp, 0x00110000,true);
        //rd_mem_cmd(tx_usrp, 0x00540000,true);

        //Configure runtime mode-------------------------------------------------------------------------------------------

        //bb-engine mode: active (pkt tx), sync 
        std::uint32_t modeBits{0x0 << 2}; //modeBits: 2 bits [src,dest]. For each, 1->active, 0->sync. ex: mode 3 =>both active
        
        //rxChSel: controls whther the src/dest are listening to the ch-emu or afe
        //2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
        //ex: mode 0 =>both digital loopback
        //ex: mode 1 =>dest rx to afe
        //ex: mode 2 =>src rx to afe
        std::uint32_t rxChSelBits{0x1 << 4}; //1->fwd analog loopback
        
        //txCoreBits: controls which engine transmits through the afe
        //2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
        //ex: mode 0 =>both digital loopback
        //ex: mode 1 =>dest afe tx
        //ex: mode 2 =>src afe tx
        std::uint32_t txCoreBits{0x0 << 6}; //2->fwd analog loopback

        file = "usrp_samples.wired.noise.dat";

        uint32_t start_cmd = 0x80010002+modeBits+rxChSelBits+txCoreBits;
        std::cout << "mode: " << (modeBits>>2) << " rxChSel: " << (rxChSelBits>>4)<< " txCore: " << (txCoreBits>>6) << std::endl; 
        //std::cout << std::hex << std::setw(8) << std::setfill('0') << start_cmd << std::endl;

        //Threshold and angle settings
        //std::cout << "Writing to regs...\n";
        for(const auto& cmd : write_cmds) {
            wr_mem_cmd(tx_usrp, cmd);
        }

        // std::cout << "Readback...\n";
        // for(const auto& cmd : read_cmds) {
        //     rd_mem_cmd(tx_usrp, cmd,true);
        // }


        std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active
        //std::cout << "Start command issued...\n";
        wr_mem_cmd(tx_usrp, start_cmd);

        // std::cout << "Reading results...\n";
        // for(const auto& cmd : read_cmds) {
        //     rd_mem_cmd(tx_usrp, cmd,true);
        // }
        // std::cout << "Done printing digital loopback results...\n";



        //On Chip Acquisition
        //Read data and write to binary file to be parsed by matlab for each preamble memory address
        std::vector<std::complex<double>> cap_samps;
        const int NumPrmblSamps = 512;
        const uint16_t PrmblStartAddrI = 0xC00, PrmblStartAddrQ = 0xE00;
        

        std::ofstream of_file(file, std::ios::binary | std::ios::trunc);
        //speed test result: 10 mil samps -> 16 hrs
        //51200 samples: nanosleep: 40 sec
        //nosleep: 30
        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();
        for(int ii = 0; ii < 100; ii++) 
        {
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
            samp *= static_cast<std::complex<double>>(pow(2, -8));
            cap_samps.push_back(samp);
            //std::cout << std::dec << i << " " << samp << std::endl;

            } else {
                std::cerr << "Unable to open file for appending." << std::endl;
            }

            }
        }

        
        // Close the file
        of_file.close();

        std::cout << "Samples written to file" << std::endl;
        int N_w = static_cast<int>(cap_samps.size()); //number of captured samples
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        // Print the result and execution time
        std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;

       
    }
*/
    

    // clean up transmit worker
    stop_signal_called = true;
    transmit_thread.join();
    recv_thread.join();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
