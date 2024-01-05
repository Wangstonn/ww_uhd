// This test is not complete at all. Just putting this here in case I need to test mmio separately

//should be in header file....
constexpr std::uint32_t cmdBits{0xF0000000}; 
    constexpr std::uint32_t addrBits{0x0FFF0000};
    constexpr std::uint32_t dataBits{0x0000FFFF};


    
    constexpr uint32_t rst_cmd = 0x80010001;

    //Load phases and thresholds
    std::vector<uint32_t> write_cmds = {
        0x80000000,
        0x80010001,
        0x80101DED,          
        //0x80110000,
        0x8020CE94,          //32'h8020FFFF,
        0x8021E12A,          //32'h8021FFFF,
        0x8050A000,
        0x80510005,
        0x80520453,
        0x80533937,
        //0x80540000, //dest delay, should be 0x80540009
        0x8060030B,
        0x80613DD1,
        0x8062223C,
        0x8063CA59
    };

    //Readback input bit, phase, and threshold settings, valid and done, and pkt out
    std::vector<uint32_t> read_cmds = {
        0x00000000,
        0x00010000,
        0x00100000,
        0x00110000,
        0x00200000,
        0x00210000,
        0x00500000,
        0x00510000,
        0x00520000,
        0x00530000,
        0x00540000,
        0x00600000,
        0x00610000,
        0x00620000,
        0x00630000,
        
        0x08000000, //valid and done
        0x08100000, //pkt out
        0x08110000, //pkt out

        0x0C000000,
        0x0DFF0000,
        0x0E000000,
        0x0FFF0000
    };

    constexpr std::uint32_t isDone{0x08000001}; //src done tx
    constexpr std::uint32_t isValid{0x08000002}; //dest done tx

    constexpr int ms_delay{1};
//Configure runtime mode-------------------------------------------------------------------------------------------

    //bb-engine mode: active (pkt tx), sync 
    std::uint32_t modeBits{0x0 << 2}; //modeBits: 2 bits [src,dest]. For each, 1->active, 0->sync. ex: mode 3 =>both active
    
    //rxChSel: controls whther the src/dest are listening to the ch-emu or afe
    //2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
    //ex: mode 0 =>both digital loopback
    //ex: mode 1 =>dest rx to afe
    //ex: mode 2 =>src rx to afe
    std::uint32_t rxChSelBits{0x1 << 4}; 
    
    //txCoreBits: controls which engine transmits through the afe
    //2 bits [src,dest]. For each, 1->afe, 0->digital channel emulator. 
    //ex: mode 0 =>both digital loopback
    //ex: mode 1 =>dest afe tx
    //ex: mode 2 =>src afe tx
    std::uint32_t txCoreBits{0x2 << 6};

    uint32_t start_cmd = 0x80010002+modeBits+rxChSelBits+txCoreBits;
    std::cout << "mode: " << (modeBits>>2) << " rxChSel: " << (rxChSelBits>>4)<< " txCore: " << (txCoreBits>>6) << std::endl; 
    //std::cout << std::hex << std::setw(8) << std::setfill('0') << start_cmd << std::endl;

    //Threshold and angle settings
    std::cout << "Writing to regs...\n";
    for(const auto& cmd : write_cmds) {
        wr_mem_cmd(tx_usrp, cmd);
    }

    std::cout << "Readback...\n";
    for(const auto& cmd : read_cmds) {
        rd_mem_cmd(tx_usrp, cmd,true);
    }
std::cout << "Start command issued...\n";
    wr_mem_cmd(tx_usrp, start_cmd);

    // // recv to file
    // bool rx_save = save_file != 0;
    // if (type == "double")
    //     recv_to_file<std::complex<double>>(
    //         rx_usrp, "fc64", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    // else if (type == "float")
    //     recv_to_file<std::complex<float>>(
    //         rx_usrp, "fc32", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    // else if (type == "short")
    //     recv_to_file<std::complex<short>>(
    //         rx_usrp, "sc16", otw, file, spb, total_num_samps, settling, rx_channel_nums, rx_save);
    // else {
    //     // clean up transmit worker
    //     stop_signal_called = true;
    //     transmit_thread.join();
    //     throw std::runtime_error("Unknown type " + type);
    // }



    std::cout << "Reading results...\n";
    for(const auto& cmd : read_cmds) {
        rd_mem_cmd(tx_usrp, cmd,true);
    }
    std::cout << "Done printing digital loopback results...\n";