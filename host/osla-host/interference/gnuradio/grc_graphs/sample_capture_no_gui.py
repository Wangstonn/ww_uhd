#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: sample capture
# GNU Radio version: 3.10.12.0

from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import math
import threading




class sample_capture_no_gui(gr.top_block):

    def __init__(self, capture_t=4, tx_freq=2.2e9):
        gr.top_block.__init__(self, "sample capture", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.capture_t = capture_t
        self.tx_freq = tx_freq

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.RX_ID = RX_ID = "addr=192.168.10.2"

        ##################################################
        # Blocks
        ##################################################

        self.uhd_usrp_source_0_0 = uhd.usrp_source(
            ",".join((RX_ID, '')),
            uhd.stream_args(
                cpu_format="sc16",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_source_0_0.set_center_freq(tx_freq, 0)
        self.uhd_usrp_source_0_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_source_0_0.set_bandwidth(160000000, 0)
        self.uhd_usrp_source_0_0.set_gain(0, 0)
        self.blocks_head_0 = blocks.head(gr.sizeof_short*2, (capture_t*samp_rate))
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_short*2, '/tmp/samnolan/c16_noise_10M.bin', False)
        self.blocks_file_sink_0.set_unbuffered(False)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_head_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.uhd_usrp_source_0_0, 0), (self.blocks_head_0, 0))


    def get_capture_t(self):
        return self.capture_t

    def set_capture_t(self, capture_t):
        self.capture_t = capture_t
        self.blocks_head_0.set_length((self.capture_t*self.samp_rate))

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_source_0_0.set_center_freq(self.tx_freq, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_head_0.set_length((self.capture_t*self.samp_rate))
        self.uhd_usrp_source_0_0.set_samp_rate(self.samp_rate)

    def get_RX_ID(self):
        return self.RX_ID

    def set_RX_ID(self, RX_ID):
        self.RX_ID = RX_ID



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--capture-t", dest="capture_t", type=eng_float, default=eng_notation.num_to_str(float(4)),
        help="Set capture time duration [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Front end Tx frequency [default=%(default)r]")
    return parser


def main(top_block_cls=sample_capture_no_gui, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(capture_t=options.capture_t, tx_freq=options.tx_freq)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
