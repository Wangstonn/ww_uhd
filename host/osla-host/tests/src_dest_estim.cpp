//
// Copyright 2010-2012,2014-2015 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
//WW-This file is a modified version of uhd/host/examples/txrx_loopback_to_file.cpp. It essentially sets up + activates the radios and writes & sends samples to file

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


/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // transmit variables to be set by po
    std::string src_args, wave_type, tx_ant, tx_subdev, ref, otw, tx_channels;
    double tx_rate, fwd_freq, tx_gain, wave_freq, tx_bw;
    float ampl;

    // receive variables to be set by po
    std::string dest_args, file, type, rx_ant, rx_subdev, rx_channels;
    size_t total_num_samps, spb, save_file;
    double rx_rate, fb_freq, rx_gain, rx_bw;
    double settling;

    //WW - optional user defined arguments
    uint32_t input_reg, output_reg;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        //usrp selection
        ("tx-args", po::value<std::string>(&src_args)->default_value("type=x300,addr=192.168.110.2"), "uhd transmit device address args")
        ("rx-args", po::value<std::string>(&dest_args)->default_value("type=x300,addr=192.168.10.2"), "uhd receive device address args")
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
        ("tx-freq", po::value<double>(&fwd_freq)->default_value(.915e9), "transmit RF center frequency in Hz")
        ("rx-freq", po::value<double>(&fb_freq)->default_value(.915e9), "receive RF center frequency in Hz")
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
    std::cout << boost::format("Creating the src usrp device with: %s...") % src_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr src_usrp = uhd::usrp::multi_usrp::make(src_args);
    std::cout << std::endl;

    std::cout << boost::format("Creating the dest usrp device with: %s...") % dest_args
              << std::endl;
    uhd::usrp::multi_usrp::sptr dest_usrp = uhd::usrp::multi_usrp::make(dest_args);

    // always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("tx-subdev")){
            src_usrp->set_tx_subdev_spec(tx_subdev);
            src_usrp->set_rx_subdev_spec(rx_subdev);
    }
    if (vm.count("rx-subdev")){
            dest_usrp->set_tx_subdev_spec(tx_subdev);
            dest_usrp->set_rx_subdev_spec(rx_subdev);
    }

    // Lock mboard clocks
    if (vm.count("ref")) {
        src_usrp->set_clock_source(ref);
        dest_usrp->set_clock_source(ref);
    }

    std::cout << "Using Src Device: " << src_usrp->get_pp_string() << std::endl;
    std::cout << "Using Dest Device: " << dest_usrp->get_pp_string() << std::endl;

    // set the transmit sample rate
    if (not vm.count("tx-rate")) {
        std::cerr << "Please specify the transmit sample rate with --tx-rate"
                  << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate / 1e6)
              << std::endl;
    src_usrp->set_tx_rate(tx_rate);
    dest_usrp->set_tx_rate(tx_rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...")
                     % (src_usrp->get_tx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the receive sample rate
    if (not vm.count("rx-rate")) {
        std::cerr << "Please specify the sample rate with --rx-rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate / 1e6)
              << std::endl;
    src_usrp->set_rx_rate(rx_rate);
    dest_usrp->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...")
                     % (src_usrp->get_rx_rate() / 1e6)
              << std::endl
              << std::endl;

    // set the transmit center frequency
    if (not vm.count("tx-freq")) {
        std::cerr << "Please specify the transmit center frequency with --tx-freq"
                  << std::endl;
        return ~0;
    }


    size_t channel = 0;
    std::cout << "Configuring TX Channel " << channel << std::endl;
    std::cout << boost::format("Setting Fwd Channel Freq: %f MHz...") % (fwd_freq / 1e6)
                << std::endl;
    uhd::tune_request_t fwd_tune_request(fwd_freq);
    uhd::tune_request_t fb_tune_request(fb_freq);
    src_usrp->set_tx_freq(fwd_tune_request, channel);
    dest_usrp->set_tx_freq(fb_tune_request, channel);

    std::cout << boost::format("Actual TX Freq: %f MHz...")
                        % (src_usrp->get_tx_freq(channel) / 1e6)
                << std::endl
                << std::endl;

    //std::cout << src_usrp->get_rx_gain_range(channel).step() << std::endl;
    // set the rf gain, ubx range: 0-31.5dB
    tx_gain = 0;
    if (vm.count("tx-gain")) {
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain
                    << std::endl;
        src_usrp->set_tx_gain(tx_gain, channel);
        dest_usrp->set_tx_gain(tx_gain, channel);
        std::cout << boost::format("Actual TX Gain: %f dB...")
                            % src_usrp->get_tx_gain(channel)
                    << std::endl
                    << std::endl;



        // set the analog frontend filter bandwidth
        if (vm.count("tx-bw")) {
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % tx_bw
                      << std::endl;
            src_usrp->set_tx_bandwidth(tx_bw, channel);
            dest_usrp->set_tx_bandwidth(tx_bw, channel);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                             % src_usrp->get_tx_bandwidth(channel)
                      << std::endl
                      << std::endl;
        }

        // set the antenna
        if (vm.count("tx-ant"))
            src_usrp->set_tx_antenna(tx_ant, channel);
            dest_usrp->set_tx_antenna(tx_ant, channel);
    }

    std::cout << "Configuring RX Channel " << channel << std::endl;

    // set the receive center frequency
    if (not vm.count("rx-freq")) {
        std::cerr << "Please specify the center frequency with --rx-freq"
                    << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting Src RX Freq: %f MHz...") % (fb_freq / 1e6)
                << std::endl;
    src_usrp->set_rx_freq(fb_tune_request, channel);
    dest_usrp->set_rx_freq(fwd_tune_request, channel);
    std::cout << boost::format("Actual Src RX Freq: %f MHz...")
                        % (src_usrp->get_rx_freq(channel) / 1e6)
                << std::endl
                << std::endl;

    // set the receive rf gain ubx range: 0-31.5dB
    if (vm.count("rx-gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain
                    << std::endl;
        src_usrp->set_rx_gain(rx_gain, channel);
        dest_usrp->set_rx_gain(rx_gain, channel);
        std::cout << boost::format("Actual src RX Gain: %f dB...")
                            % src_usrp->get_rx_gain(channel)
                    << std::endl
                    << std::endl;
    }

    // set the receive analog frontend filter bandwidth
    if (vm.count("rx-bw")) {
        std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_bw / 1e6)
                    << std::endl;
        src_usrp->set_rx_bandwidth(rx_bw, channel);
        dest_usrp->set_rx_bandwidth(rx_bw, channel);
        std::cout << boost::format("Actual src RX Bandwidth: %f MHz...")
                            % (src_usrp->get_rx_bandwidth(channel) / 1e6)
                    << std::endl
                    << std::endl;
    }

    // set the receive antenna
    if (vm.count("rx-ant"))
        src_usrp->set_rx_antenna(rx_ant, channel);
        dest_usrp->set_rx_antenna(rx_ant, channel);

    //print options
    std::vector<std::string> gpio_banks;
    gpio_banks = src_usrp->get_gpio_banks(0);
    for(const auto& bank : gpio_banks) {
        std::cout << bank << std::endl;
    }
    
    // for the const wave, set the wave freq for small samples per period
    if (wave_freq == 0 and wave_type == "CONST") {
        wave_freq = src_usrp->get_tx_rate() / 2;
        wave_freq = dest_usrp->get_tx_rate() / 2;
    }

    // error when the waveform is not possible to generate
    if (std::abs(wave_freq) > src_usrp->get_tx_rate() / 2) {
        throw std::runtime_error("wave freq out of Nyquist zone");
    }
    if (src_usrp->get_tx_rate() / std::abs(wave_freq) > wave_table_len / 2) {
        throw std::runtime_error("wave freq too small for table");
    }

    // pre-compute the waveform values
    const wave_table_class wave_table(wave_type, ampl);
    const size_t step = std::lround(wave_freq / src_usrp->get_tx_rate() * wave_table_len);
    size_t index      = 0;

    // create a transmit streamer
    // linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32", otw);
    stream_args.channels             = {0};
    uhd::tx_streamer::sptr src_tx_stream = src_usrp->get_tx_stream(stream_args);
    uhd::tx_streamer::sptr dest_tx_stream = dest_usrp->get_tx_stream(stream_args);

    // allocate a buffer which we re-use for each channel
    if (spb == 0)
        spb = src_tx_stream->get_max_num_samps() * 10;
    std::vector<std::complex<float>> src_buff(spb);
    std::vector<std::complex<float>> dest_buff(spb);
    int num_channels = 1;

    // setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(0.5); // give us 0.5 seconds to fill the tx buffers

    // Check Ref and LO Lock detect
    std::vector<std::string> tx_sensor_names, rx_sensor_names;
    tx_sensor_names = src_usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = src_usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    rx_sensor_names = src_usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = src_usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    tx_sensor_names = dest_usrp->get_tx_sensor_names(0);
    if (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "lo_locked")
        != tx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = src_usrp->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    rx_sensor_names = dest_usrp->get_rx_sensor_names(0);
    if (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "lo_locked")
        != rx_sensor_names.end()) {
        uhd::sensor_value_t lo_locked = src_usrp->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    tx_sensor_names = src_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = src_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = src_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    rx_sensor_names = src_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = src_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = src_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    //
        tx_sensor_names = dest_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "mimo_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = dest_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(tx_sensor_names.begin(), tx_sensor_names.end(), "ref_locked")
                != tx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = dest_usrp->get_mboard_sensor("ref_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    rx_sensor_names = dest_usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "mimo_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t mimo_locked = dest_usrp->get_mboard_sensor("mimo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % mimo_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(mimo_locked.to_bool());
    }
    if ((ref == "external")
        and (std::find(rx_sensor_names.begin(), rx_sensor_names.end(), "ref_locked")
                != rx_sensor_names.end())) {
        uhd::sensor_value_t ref_locked = dest_usrp->get_mboard_sensor("ref_locked", 0);
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
    src_usrp->set_time_now(uhd::time_spec_t(0.0));
    dest_usrp->set_time_now(uhd::time_spec_t(0.0));

    src_usrp->set_rx_dc_offset(true);
    dest_usrp->set_rx_dc_offset(true);

        
    //Start tx and streaming
    // start transmit worker thread
    std::thread src_transmit_thread([&]() {
        transmit_worker(src_buff, wave_table, src_tx_stream, md, step, index, num_channels); //this sets tx_streamer which gates tx
    });
    std::thread dest_transmit_thread([&]() {
        transmit_worker(dest_buff, wave_table, dest_tx_stream, md, step, index, num_channels); //this sets tx_streamer which gates tx
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active

    // recv to file - supposedly sets registers on adc but I cant find anything about that. 
    // However, given how transmit_worker sets the tx settings (tx_running), its very possible that rx settings need to be set for proper operation
    // Ordinary operation of recv_to_file will lock out the rest of the c++ code, so try putting it in a thread so that it can execute indefinitely just like transmit_worker
    // This will block streaming though. If you want to record samples you that will have to modify recv_to_file to write to file for only part of the time recv to file is active.
    //  Or separately call this after a run is complete to capture strobed data...
    std::thread src_recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            src_usrp, "fc64", otw, file, spb, total_num_samps, settling, {0}, 0); //save_rx = 0 so that we dont create a huge file
    });
    std::thread dest_recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            dest_usrp, "fc64", otw, file, spb, total_num_samps, settling, {0}, 0); //save_rx = 0 so that we dont create a huge file
    });


    //--------------------------------------------------
    //WW - OSLA-BPSK Operation
    //--------------------------------------------------

    //Preload some default threshold and angle settings
    mmio::InitBBCore(src_usrp);

    //test settings
    std::uint32_t tx_core_bits{0b10}; 
    std::uint32_t rx_ch_sel_bits{0b01}; 
    std::uint32_t gpio_start_sel_bits{0b00};

    //noise estimation-----------------------------------------------------------------------------------------------------------------------
    std::cout << "Running noise estimation..." << std::endl;
    double var = estim::EstimChipNoise(src_usrp, pow(2,15),rx_ch_sel_bits, "../../data/fwd_alb_noise_samps.dat"); //../../data/fwd_alb_noise_samps.dat
    std::cout << "Estimated var= " << var << std::endl;

    // Timing+flatfading estimation---------------------------------------------------------------------------------------------------------------------------
    //Because our window is small, need to sweep multiple time intervals by adjusting source and dest delay. Assumes channel coherence is quite long
    //Multiple tests have confirmed wired loopback delay with 8inch sma cable + attenuator is 119, so its find to just do one interval for now
    std::cout << "Running delay+flatfading estimation..." << std::endl;

    double EsN0;
    int D_hat;
    std::complex<double> h_hat;

    int D_test = 0;
    
    auto ch_params = estim::ChEstim(src_usrp, D_test, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits, pow(2,15), "../../data/fwd_alb_prmbl_samps.dat"); //../../data/fwd_alb_prmbl_samps.dat
        D_hat = ch_params.D_hat;
        h_hat = ch_params.h_hat;
    EsN0 = estim::CalcChipEsN0(h_hat, var);

    std::cout << std::dec << "D_test= " << D_test << ", ";
    std::cout << "D_hat= " << D_hat << ", ";
    std::cout << "EsN0= " << EsN0 << ", ";
    std::cout << "h_hat : abs= " << std::abs(h_hat) << " arg= " << std::arg(h_hat) << std::endl;

    //Test setup------------------------------------------------------------------
    std::cout << "Performing compensation..." << std::endl;
    int D_eff = D_hat;
    estim::CompensateDelays(src_usrp, D_eff);

    std::complex<double> h_comp = h_hat/std::abs(h_hat); 
    estim::PhaseEq(src_usrp, h_comp);




    // //BER test------------------------------------------------------------------------------------------------------
    // mode_bits = 0b11;
    // rx_ch_sel_bits = 0b01; 
    // tx_core_bits = 0b10; 
    // gpio_start_sel_bits = 0b00;

    // double n_errors{0}; 

    // const int kMaxIter = 1e6;
    // const int kTargetErr = 100;
    // //Generate input bits
    // std::random_device rd;

    // // Create a Mersenne Twister PRNG engine
    // std::mt19937 mt(rd());

    // // Define a distribution for generating uint32_t values
    // std::uniform_int_distribution<uint32_t> dist;
    
    // for(int iter = 1; iter < kMaxIter; iter++ ) {
    //     // Generate a random pkt
    //     const int Num16BitSlices = mmio::kPktLen/32;
    //     uint32_t input_pkt[Num16BitSlices] = {0};
    //     uint32_t output_pkt[Num16BitSlices] = {0};

    //     // Generate a random uint32_t
    //     for(int i = 0; i < Num16BitSlices; i++)
    //     {
    //         uint32_t randomValue = dist(mt);
    //         //std::cout << "Random uint32_t: " << std::hex << std::setw(4) << std::setfill('0') << randomValue << std::endl;

    //         input_pkt[i] = randomValue;

    //         mmio::WrMmio(src_usrp, mmio::kInPktAddr+i, randomValue);
    //     }

    //     // start
    //     mmio::start_tx(src_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits);
        
    //     while(true)
    //     {
    //         //Run and check received pkt    
    //         mmio::WrMmio(src_usrp,0x0,0x0); //need to clear addr buffer, not sure why its 0x8. 0x0 should work fine...
    //         bool pkt_valid = mmio::rd_mem_cmd(src_usrp, mmio::kBbStatusAddr) & 0x2; //around 10 ms
    //         if(pkt_valid)
    //             break;
    //     }

    //     // read results ---------------------------------------------
    //     for(int i = 0; i*32 < mmio::kPktLen; i++) {
    //         output_pkt[i] = mmio::rd_mem_cmd(src_usrp, mmio::kOutPktAddr+i);
    //         //std::cout << std::hex << input_pkt[i] << std::endl;

    //         uint32_t xor_result = output_pkt[i] ^ input_pkt[i];
    //         while (xor_result > 0) {
    //             n_errors += xor_result & 1;
    //             xor_result >>= 1;
    //         }

    //         // std::cout << std::dec << "Bit slice: " << i << std::endl;
    //         // std::cout << std::hex << "Input:  " << input_pkt[i] << std::endl;
    //         // std::cout << std::hex << "Output: " << output_pkt[i] << std::endl << std::endl;
    //     }

    //     if(n_errors > kTargetErr){
    //         // ber_array.push_back(n_errors/(iter*mmio::kPktLen));
    //         // n_bit_err_array.push_back(n_errors);
    //         // n_iter_array.push_back(iter*mmio::kPktLen);
    //         std::cout << std::dec << "Reached " << n_errors << " errors for EsN0 = " << target_EsN0 << " in " << iter << " iterations" << std::endl;
    //         std::cout << "ber =" << n_errors/(iter*mmio::kPktLen) << std::endl;
    //         break;
    //     }
    // }


    //}

    // std::cout << "Completed experiment with following BER results" << std::endl;

    // // Print MATLAB command to instantiate an array with vector contents
    // std::cout << "ber = ["; // Start MATLAB array definition
    // // Loop through the vector and print each element
    // for (size_t i = 0; i < ber_array.size(); ++i) {
    //     std::cout << ber_array[i]; // Print the current element

    //     // If not the last element, print a comma and space
    //     if (i != ber_array.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "];" << std::endl; // End MATLAB array definition

    // std::cout << "EsN0 = ["; // Start MATLAB array definition
    // for (size_t i = 0; i < EsN0_array.size(); ++i) {
    //     std::cout << EsN0_array[i]; // Print the current element

    //     // If not the last element, print a comma and space
    //     if (i != EsN0_array.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "];" << std::endl; // End MATLAB array definition

    // std::cout << "n_bit_err = ["; // Start MATLAB array definition
    // for (size_t i = 0; i < n_bit_err_array.size(); ++i) {
    //     std::cout << n_bit_err_array[i]; // Print the current element

    //     // If not the last element, print a comma and space
    //     if (i != n_bit_err_array.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "];" << std::endl; // End MATLAB array definition    

    // std::cout << "n_iter_array = ["; // Start MATLAB array definition
    // for (size_t i = 0; i < n_iter_array.size(); ++i) {
    //     std::cout << n_iter_array[i]; // Print the current element

    //     // If not the last element, print a comma and space
    //     if (i != n_iter_array.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << "];" << std::endl; // End MATLAB array definition    
    //////////////////////////////////////////////////////////////////////////////////////////////////

    // clean up transmit worker
    stop_signal_called = true;
    src_transmit_thread.join();
    dest_transmit_thread.join();
    src_recv_thread.join();
    dest_recv_thread.join();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
