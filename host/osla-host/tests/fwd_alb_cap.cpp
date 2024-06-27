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

        //std::cout << tx_usrp->get_rx_gain_range(channel).step() << std::endl;
        // set the rf gain, ubx range: 0-31.5dB
        tx_gain = 0;
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

        // set the receive rf gain ubx range: 0-31.5dB
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

    //print options
    std::vector<std::string> gpio_banks;
    gpio_banks = tx_usrp->get_gpio_banks(0);
    for(const auto& bank : gpio_banks) {
        std::cout << bank << std::endl;
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
    //For early termination use Ctrl + Z

    // reset usrp time to prepare for transmit/receive
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    tx_usrp->set_time_now(uhd::time_spec_t(0.0));


    //--------------------------------------------------
    //WW - OSLA-BPSK Operation
    //--------------------------------------------------

    tx_usrp->set_rx_dc_offset(true);
    //Preload some default threshold and angle settings
    mmio::InitBBCore(tx_usrp);
        
    //Start tx and streaming
    // start transmit worker thread
    std::thread transmit_thread([&]() {
        transmit_worker(buff, wave_table, tx_stream, md, step, index, num_channels); //this sets tx_streamer which gates tx
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Need to sleep for at least 500 ms before tx is active

    // recv to file - supposedly sets registers on adc but I cant find anything about that. 
    // However, given how transmit_worker sets the tx settings (tx_running), its very possible that rx settings need to be set for proper operation
    // Ordinary operation of recv_to_file will lock out the rest of the c++ code, so try putting it in a thread so that it can execute indefinitely just like transmit_worker
    // This will block streaming though. If you want to record samples you that will have to modify recv_to_file to write to file for only part of the time recv to file is active.
    //  Or separately call this after a run is complete to capture strobed data...
    std::thread recv_thread([&]() {
        recv_to_file<std::complex<double>>(
            rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, rx_channel_nums, 0); //save_rx = 0 so that we dont create a huge file
    });

    //test settings
    std::uint32_t tx_core_bits{0b10}; 
    std::uint32_t rx_ch_sel_bits{0b01}; 
    std::uint32_t gpio_start_sel_bits{0b00};

    //noise estimation
    double var;
    std::cout << "Running noise estimation..." << std::endl;
    var = estim::EstimNoise(tx_usrp, pow(2,15),rx_ch_sel_bits); //use 2^15 samples to get a good estimate for the noise
    std::cout << "Estimated var= " << var << std::endl; //one time this gave me a negative....

    // Timing+flatfading estimation---------------------------------------------------------------------------------------------------------------------------
    //Because our window is small, need to sweep multiple time intervals by adjusting source and dest delay. Assumes channel coherence is quite long
    //Multiple tests have confirmed wired loopback delay with 8inch sma cable + attenuator is 119, so its find to just do one interval for now
    std::cout << "Running delay+flatfading estimation..." << std::endl;

    std::vector<int> D_hat_sweep, D_test_sweep;
    std::vector<std::complex<double>> h_hat_sweep;
    std::vector<double> SNR_sweep;

    int N_sweep_intervals = 0; //5
    const int DelaySweepInterval = 512;

    for(int interval_idx = -N_sweep_intervals/2; interval_idx <= N_sweep_intervals/2; interval_idx++) {
        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        int D_test = interval_idx * DelaySweepInterval; //D_test is the delay between src and dest we set. This is the opposite of D_comp's logic

        auto ch_params = estim::ChEstim(tx_usrp, D_test, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits, pow(2,12), "");
        int D_hat = ch_params.D_hat;
        std::complex<double> h_hat = ch_params.h_hat;

        double SNR = estim::CalcSNR(h_hat, var);

        D_test_sweep.push_back(D_test);
        D_hat_sweep.push_back(D_hat);
        h_hat_sweep.push_back(h_hat);
        SNR_sweep.push_back(SNR);

        
        //std::cout << "r[max_idx] = " << r[max_idx] << ", "; 
        
        std::cout << "D_test_sweep = " << D_test << ", ";
        std::cout << "D_hat_sweep" << " = " << D_hat << ", ";
        std::cout << "SNR_sweep" << " = " << SNR << ", ";
        std::cout << "h_hat_sweep" << " : abs= " << std::abs(h_hat) << " arg= " << std::arg(h_hat) << std::endl;

        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate the elapsed time
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

        // Print the result and execution time
        std::cout << "Execution time: " << duration.count() << " seconds" << std::endl;
    }

    // Find the index of the maximum absolute value in vector r
    auto max_it = std::max_element(h_hat_sweep.begin(), h_hat_sweep.end(), [](const std::complex<double>& a, const std::complex<double>& b) {
        return std::abs(a) < std::abs(b);
    }); //Finds the iterator pointing to the max element
    int max_idx = std::distance(h_hat_sweep.begin(), max_it); //Finds the index corresponding to that iterator

    std::complex<double> h_hat = h_hat_sweep[max_idx];
    int D_hat = D_hat_sweep[max_idx];
    double SNR = SNR_sweep[max_idx];
    double EsN0 = 0;
    if(true) {
        //redo timing/flatfade estimation using the estimated delay to get the full preamble----------------------------------------
        int D_test = D_hat;
        
        auto ch_params = estim::ChEstim(tx_usrp, D_test, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits, mmio::kCapMaxNumSamps, "");
        D_hat = ch_params.D_hat;
        h_hat = ch_params.h_hat;
        SNR = estim::CalcSNR(h_hat, var);
        EsN0 = estim::CalcEsN0(h_hat, estim::kNChips*estim::kFwOsr, var);

        std::cout << std::dec;
        std::cout << "D_test= " << D_test << ", ";
        std::cout << "D_hat= " << D_hat << ", ";
        std::cout << "SNR= " << SNR << ", ";
        std::cout << "EsN0= " << EsN0 << ", ";
        std::cout << "h_hat : abs= " << std::abs(h_hat) << " arg= " << std::arg(h_hat) << std::endl;
    }
    
    // //Gain Control------------------------------------------------------------------
    // // Max SNR test 
    // std::cout << "Performing compensation..." << std::endl;
    // //Use the high SNR estimate because its more reliable. Set the digital gain to 1
    // std::complex<double> h_comp = h_hat/std::abs(h_hat);
    // //Phase Compensation
    // estim::PhaseEq(tx_usrp, h_comp);

    // int D_eff = D_hat + 4;
    // estim::CompensateDelays(tx_usrp, D_eff);

    // //Basic analog loopback test   
    // // Digital gain is [0.66,2]. Set signal amplitude to 1. 
    // double h_mag = std::abs(h_hat);
    // double total_gain = -20*std::log10(h_mag); //total gain needed in system to equalize 
    // std::cout << "total gain: " << total_gain << std::endl;
    // tx_gain = 0;
    // rx_gain = 0;

    // double digital_gain = total_gain;
    // double lin_digital_gain = std::pow(10,digital_gain/20);
    // uint16_t tx_amp = static_cast<uint16_t>(std::round(lin_digital_gain*(std::pow(2,15)-1)));
    // mmio::WrMmio(tx_usrp,mmio::kSrcTxAmpAddr,tx_amp);

    //analog loopback test at specific snr ----------------------------------------------
    double tx_gain_base = tx_usrp->get_tx_gain(0); //base tx gain used for smaple capture
    //assume tx amp is max
    //assume rx gain is 0
    std::complex<double> h_phase = h_hat/std::abs(h_hat); //save the unit magnitude component

    //Set operating EsN0
    double target_EsN0 = 5; //in dB
    std::cout << "Target Es_N0 = " << target_EsN0 << std::endl;
    double target_gain = target_EsN0-EsN0; //Target gain needed to test system
    std::cout << "target gain:" << target_gain << std::endl;
    //if target gain > 0, boost tx gain.
    if(target_gain > 0) {
        tx_gain = tx_gain_base + std::ceil(target_gain * 2) / 2.0; //round up to nearest half integer. Then decrease tx amp to achieve desired EsN0
        if(tx_gain > 31.5){
            std::cerr << "Error: target EsN0 is out of tx_gain range. The maximum tx power is not enough"  << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
        tx_usrp->set_tx_gain(tx_gain, 0); //only using channel 0
        std::cout << boost::format("Actual TX Gain: %f dB...") % tx_usrp->get_tx_gain(0) << std::endl << std::endl;

        //decrease tx_amp to achieve desired EsN0
        double lin_digital_gain = std::pow(10,(target_gain-(tx_gain-tx_gain_base))/20);
        uint16_t tx_amp = static_cast<uint16_t>(std::round(lin_digital_gain*(std::pow(2,15)-1)));
        mmio::WrMmio(tx_usrp,mmio::kSrcTxAmpAddr,tx_amp);

    } else {
        tx_gain = std::max(std::ceil((tx_gain_base + target_gain) * 2) / 2.0, 0.0); //decrease tx_gain until just above target, then decrease tx amp
        std::cout << boost::format("Setting TX Gain: %f dB...") % tx_gain << std::endl;
        tx_usrp->set_tx_gain(tx_gain, 0); //only using channel 0
        std::cout << boost::format("Actual TX Gain: %f dB...") % tx_usrp->get_tx_gain(0) << std::endl << std::endl;
        
        double lin_digital_gain = std::pow(10,(target_gain-(tx_gain-tx_gain_base))/20);
        uint16_t tx_amp = static_cast<uint16_t>(std::round(lin_digital_gain*(std::pow(2,15)-1)));
        std::cout << std::hex << "tx_amp:" << tx_amp << std::endl;
        mmio::WrMmio(tx_usrp,mmio::kSrcTxAmpAddr,tx_amp);

        if(tx_amp == 0) {
            std::cerr << "Error: tx_amp is zero. The tx power is too strong"  << std::endl;
            return EXIT_FAILURE;
        }
    }
    //Rounding will not have a significant impact. rouding introduces an error of .5 for a signal with max value 2^16-1. Consider 2^8. Adding 1 to it has 
    // adds .01 db. This has a bigger impact if tx_amp is very low, but that only occurs in the case that we have A LOT of tx power (aka a very clean channel).
    // This wont occur in any case with tx_amp > 0 since then tx-amp will be decreased by at most 3db (to 2^8).

    //Set rx gain so that signal is amplitude 1
    //Digital gain is [0.66,2]. 
    //First set tx gain to achieve higher than target SNR. Then use analog gain/tx amplitude/digital gain to set signal amplitude to 1. 
    double h_mag = std::abs(h_hat);
    std::cout << "h_hat: " << h_hat << std::endl;
    double h_mag_comp;
    mmio::WrMmio(tx_usrp, mmio::kDestNumBitShift, 0x3); //shift dest rx by 3 to the left (multiply by 8)

    double target_rx_gain = -(20*std::log10(h_mag) + target_gain) - 20*std::log10(8); //total gain needed in system to equalize 
    std::cout << std::dec << "target_rx_gain: " << target_rx_gain << std::endl;

    if(target_rx_gain > 0) {
        rx_gain = std::ceil(target_rx_gain * 2) / 2.0;
        if (rx_gain > 31.5) {
            std::cerr << "Error: target EsN0 is out of rx_gain range. The maximum tx power is not enough or the channel is not noisy enough"  << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
        rx_usrp->set_rx_gain(rx_gain, 0); //only using channel 0
        std::cout << boost::format("Actual RX Gain: %f dB...") % rx_usrp->get_rx_gain(0) << std::endl << std::endl;
    } 
    else {
        rx_gain = 0;
        std::cout << boost::format("Setting RX Gain: %f dB...") % rx_gain << std::endl;
        rx_usrp->set_rx_gain(rx_gain, 0); //only using channel 0
        std::cout << boost::format("Actual RX Gain: %f dB...") % rx_usrp->get_rx_gain(0) << std::endl << std::endl;
    }

    h_mag_comp = std::pow(10,(target_rx_gain-rx_gain)/20);
    std::cout << "h_mag_comp: " << h_mag_comp << std::endl;

    if(h_mag_comp < 2.0/3.0 || h_mag_comp > 2) {
        std::cerr << "Error: Digital compensation of h_mag is out of rand [0.66,2]. rx-gain is not set correctly or signal power is too high"  << std::endl;
        return EXIT_FAILURE;
    }
    
    //Coefficient compensation
    std::cout << "Performing compensation..." << std::endl;
    //Use the high SNR estimate because its more reliable. Set the digital gain to 1
    h_hat = 1/h_mag_comp*h_phase; //Calculate current channel coefficient (with tx/rx gains)
    std::cout << "h_hat: " << h_hat << std::endl;
    //Phase Compensation
    estim::PhaseEq(tx_usrp, h_hat); 

    mmio::RdMmio(tx_usrp,mmio::kDestChEqReAddr, true);
    mmio::RdMmio(tx_usrp,mmio::kDestChEqImAddr, true);
    
    int D_eff = D_hat + 4;
    estim::CompensateDelays(tx_usrp, D_eff);


    //Run test------------------------------------------------------------------------------------
    std::uint32_t mode_bits{0b11};

    double n_errors = 0; 

    // Generate a random pkt
    const int Num16BitSlices = mmio::kPktLen/32;
    uint32_t input_pkt[Num16BitSlices] = {0};
    uint32_t output_pkt[Num16BitSlices] = {0};

    // Generate a random uint32_t
    for(int i = 0; i < Num16BitSlices; i++)
    {
        input_pkt[i] = 0x0;

        mmio::WrMmio(tx_usrp, mmio::kInPktAddr+i, input_pkt[i]);

        mmio::RdMmio(tx_usrp, mmio::kInPktAddr+i, true);
    }

    //tx_usrp->set_rx_dc_offset(false);
    //std::this_thread::sleep_for(std::chrono::milliseconds(10000)); //Test DC offset stability
    tx_usrp->set_rx_dc_offset(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(40)); //DC offset calibration time, minimum is 33.6 ms
    tx_usrp->set_rx_dc_offset(false);
    // start
    mmio::StartTx(tx_usrp, mode_bits, rx_ch_sel_bits, tx_core_bits, gpio_start_sel_bits);
    
    while(true)
    {
        //Run and check received pkt    
        mmio::WrMmio(tx_usrp,0x0,0x0); //need to clear addr buffer, not sure why its 0x8. 0x0 should work fine...
        bool pkt_valid = mmio::RdMmio(tx_usrp, mmio::kBbStatusAddr) & 0x2; //around 10 ms
        if(pkt_valid)
            break;
    }

    // read results ---------------------------------------------
    for(int i = 0; i*32 < mmio::kPktLen; i++) {
        output_pkt[i] = mmio::RdMmio(tx_usrp, mmio::kOutPktAddr+i);
        //std::cout << std::hex << input_pkt[i] << std::endl;

        uint32_t xor_result = output_pkt[i] ^ input_pkt[i];
        while (xor_result > 0) {
            n_errors += xor_result & 1;
            xor_result >>= 1;
        }

        std::cout << std::dec << "Bit slice: " << i << " Num errors: "<< n_errors <<std::endl;
        std::cout << std::hex << "Input:  " << input_pkt[i] << std::endl;
        std::cout << std::hex << "Output: " << output_pkt[i] << std::endl << std::endl;
    }

    std::cout << std::dec << "Reached " << n_errors << " errors"<< std::endl;

    mmio::ReadSampleMem(tx_usrp, 1, std::pow(2,16)-1, "../../data/fwd_alb_samps.dat"); 
    std::cout << "Sample written to fwd_alb_samps.dat" << std:: endl; 
    
    mmio::ReadSampleMem(tx_usrp, 0, std::pow(2,16)-1, "../../data/fb_alb_samps.dat"); 

    //----------------------------------------------------------------------------------
    // clean up transmit worker
    stop_signal_called = true;
    transmit_thread.join();
    recv_thread.join();

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
