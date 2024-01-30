#include "estim.h"
#include "../mmio/mmio.h"
#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

/**
 * Takes a given source-destination delay and compensates by matching the start delay. To purposefully insert a delay of D, set D_hat = -D
 * 
 * @param D_hat estimated relative delay of source wrt destination
 * 
*/
void compensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat) {
    uint16_t dest_delay, src_delay;
    if (D_hat < 0) { // D < 0 -> prmbl early -> delay src
        dest_delay = 0;
        src_delay = -D_hat;
    }
    else { // D > 0 -> prmbl late -> delay dest
        dest_delay = D_hat;
        src_delay = 0;
    }
    wr_mem_cmd(tx_usrp, (src_delay_cmd << 32) | src_delay);
    wr_mem_cmd(tx_usrp, (dest_delay_cmd << 32) | dest_delay);
}