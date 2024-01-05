// Code chunk with code used to perform estimation and compensation of channel in c++
    //On Chip Acquisition test
    //Read data and write to binary file to be parsed by matlab for each preamble memory address
    const int NumPrmblSamps = 512;
    const uint16_t PrmblStartAddrI = 0xC00, PrmblStartAddrQ = 0xE00;
    std::vector<std::complex<double>> cap_samps;

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
           samp *= static_cast<std::complex<double>>(pow(2, -8));
           cap_samps.push_back(samp);
           //std::cout << std::dec << i << " " << samp << std::endl;

        } else {
            std::cerr << "Unable to open file for appending." << std::endl;
        }

    }
    
    // Close the file
    of_file.close();

    std::cout << "Samples written to file" << std::endl;
    int N_w = static_cast<int>(cap_samps.size()); //number of captured samples

    //On device compensation
    //Load preamble
    std::ifstream if_file("../../../matlab/mlsr/preamble.mem"); // Open the file. Notice that this is the relative path from the executable location!!

    if (!if_file.is_open()) {
        std::cerr << "Error opening the preamble file!" << std::endl;
        return 1;
    }

    // Read data from the file and store it as individual bits in a vector
    std::vector<int> preamble_bits;
    char bit;
    while (if_file >> bit) {
        preamble_bits.push_back(bit - '0'); // Convert character to integer (0 or 1)
    }

    int N_prmbl = static_cast<int>(preamble_bits.size());
    if_file.close(); // Close the file

    // Calculate prmbl_amp
    double prmbl_amp = (1 - std::pow(2, -15));

    // Calculate prmbl_samps
    std::vector<std::complex<double>> prmbl_samps;
    for (int i = 0; i < N_prmbl; ++i) {
        std::complex<double> val = {2 * (preamble_bits[i] - 0.5) * prmbl_amp,0};
        prmbl_samps.push_back(val);
    }

    // // Display the values calculated
    // std::cout << "N_prmbl: " << N_prmbl << std::endl;
    // std::cout << "prmbl_amp: " << prmbl_amp << std::endl;
    // std::cout << "prmbl_samps: ";
    // for (int i = 0; i < N_prmbl; ++i) {
    //     std::cout << prmbl_samps[i] << " ";
    // }
    // std::cout << std::endl;
    
    
    std::vector<std::complex<double>> r;
    std::vector<int> lags;
    xcorr_slow(prmbl_samps,cap_samps, r, lags);

    // Find the index of the maximum absolute value in vector r
    auto max_it = std::max_element(r.begin(), r.end(), [](const std::complex<double>& a, const std::complex<double>& b) {
        return std::abs(a) < std::abs(b);
    }); //Finds the iterator pointing to the max element
    int max_idx = std::distance(r.begin(), max_it); //Finds the index corresponding to that iterator

    // Calculate D_hat (lag at max_idx)
    int D_hat = lags[max_idx];


    // // Output the cross-correlation values and corresponding lags
    // std::cout << "Cross-correlation result:" << std::endl;
    // for (size_t i = 0; i < r.size(); ++i) {
    //     std::cout << std::dec << "Lag: " << lags[i] << ", Correlation: " << r[i] << std::endl;
    // }
    

    // Our estimate depends on the number of samples captured in the capture window (of size N_w)
    int N_samps_cap = 0;
    if (D_hat > 0) {
        N_samps_cap = N_w - D_hat;
    } else if (D_hat < N_w - N_prmbl) {
        N_samps_cap = N_prmbl + D_hat;
    } else {
        N_samps_cap = N_w;
    }

    // Calculate h_hat, h_hat_mag, and phi_hat
    std::complex<double> h_hat = r[max_idx] / (N_samps_cap * std::pow(prmbl_amp, 2));

    if (max_xcorr < std::abs(h_hat))
    {
        max_xcorr = std::abs(h_hat);
        max_chunk = interval_idx;
    }

    //matches matlab
    // std::cout << std::dec;
    // std::cout << "Number of captured samples: " << N_w << std::endl;
    // std::cout << "D_hat: " << D_hat << std::endl;
    // std::cout << "N_samps_cap: " << N_samps_cap << std::endl;
    // std::cout << "h_hat: " << h_hat << std::endl;
    // std::cout << "h_hat_mag: " << std::abs(h_hat) << std::endl;
    // std::cout << "phi_hat: " << std::arg(h_hat) << std::endl;

    //compensation values
    // Get the real and imaginary components of 1/h_hat
    std::complex<double> reciprocal_h_hat = 1.0 / h_hat;
    std::cout << reciprocal_h_hat << std::endl;
    double dest_ch_eq_re = std::real(reciprocal_h_hat)*std::pow(2, 13);
    double dest_ch_eq_im = -std::imag(reciprocal_h_hat)*std::pow(2, 13);

    //covert to bit command
    int16_t dest_ch_eq_re_int16, dest_ch_eq_im_int16, dest_ch_eq_re_int14, dest_ch_eq_im_int14;
    dest_ch_eq_re_int16 = static_cast<int16_t>(std::round(dest_ch_eq_re));
    dest_ch_eq_im_int16 = static_cast<int16_t>(std::round(dest_ch_eq_im));

    // Apply saturation check
    if (dest_ch_eq_re > INT16_MAX || dest_ch_eq_re < INT16_MIN) {
        dest_ch_eq_re_int16 = (dest_ch_eq_re > INT16_MAX) ? INT16_MAX : INT16_MIN;
        std::cout << "WARNING: dest_ch_eq saturated" << std::endl;
    }
    if (dest_ch_eq_im > INT16_MAX || dest_ch_eq_im < INT16_MIN) {
        dest_ch_eq_im_int16 = (dest_ch_eq_im > INT16_MAX) ? INT16_MAX : INT16_MIN;
        std::cout << "WARNING: dest_ch_eq saturated" << std::endl;
    }

    dest_ch_eq_re_int14 = dest_ch_eq_re_int16 >> 2;
    dest_ch_eq_im_int14 = dest_ch_eq_im_int16 >> 2;
    // std::cout << std::hex << std::setw(4) ;
    //std::cout << (dest_ch_eq_re_int16 >> 2) << std::endl;        //0x80520453,
    //std::cout << (dest_ch_eq_im_int16 >> 2)  << std::endl;        //0x80533937,
    // //3937 is approx f93d, since last two bits are ignored by device
    
    std::cout << std::hex;
    std::cout << ((0x80520453 & ~dataBits)|dest_ch_eq_re_int14) << std::endl;
    std::cout << ((0x80533937 & ~dataBits)|dest_ch_eq_im_int14) << std::endl;
    wr_mem_cmd(tx_usrp, (0x80520453 & ~dataBits) | dest_ch_eq_re_int14);
    wr_mem_cmd(tx_usrp, (0x80533937 & ~dataBits) | dest_ch_eq_im_int14);


    int D_eff = D_hat + 4;
    uint16_t dest_delay_comp, src_delay_comp;
    if (D_eff < 0) { //estimated D is negative->dest starts first since preamble is early
        dest_delay_comp = -D_eff;
        src_delay_comp = 0;
    }
    else { //estimated D is positive->src starts first since preamble is late
        dest_delay_comp = 0;
        src_delay_comp = D_eff;
    }

    std::cout << ((0x80110000 & ~dataBits) | dest_delay_comp) << std::endl;
    std::cout << ((0x80540000 & ~dataBits) | src_delay_comp) << std::endl;
    wr_mem_cmd(tx_usrp, (0x80110000 & ~dataBits) | dest_delay_comp);
    wr_mem_cmd(tx_usrp, (0x80540000 & ~dataBits) | src_delay_comp);