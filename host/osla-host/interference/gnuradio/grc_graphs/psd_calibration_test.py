#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: psd_calibration_test
# Author: Winston Wang
# Description: Transmits constant interference for a target noise level. Used to verify the psd logic is correct.
# GNU Radio version: 3.10.12.0

from gnuradio import analog
from gnuradio import blocks
import math
from gnuradio import blocks, gr
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
import psd_calibration_test_epy_block_0 as epy_block_0  # embedded python block
import threading




class psd_calibration_test(gr.top_block):

    def __init__(self, P_received=(-44.64), P_target=(-126), ch_gain=20, selector=0, tx_freq=2.2e9):
        gr.top_block.__init__(self, "psd_calibration_test", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.P_received = P_received
        self.P_target = P_target
        self.ch_gain = ch_gain
        self.selector = selector
        self.tx_freq = tx_freq

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.TX_ID = TX_ID = "addr=192.168.10.2"
        self.F_Of = F_Of = 0
        self.F_If = F_If = 595238

        ##################################################
        # Blocks
        ##################################################

        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join((TX_ID, '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.uhd_usrp_sink_0.set_clock_source('external', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_sink_0.set_center_freq(tx_freq, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_bandwidth(160000000, 0)
        self.uhd_usrp_sink_0.set_gain(ch_gain, 0)
        self.epy_block_0 = epy_block_0.blk(fs=10000000.0, P_received=-44.64, P_target=-126.84, bw=18500.0, fc=F_If + F_Of, selector=selector)
        self.blocks_message_debug_0 = blocks.message_debug(True, gr.log_levels.info)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_If/samp_rate)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, 1, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.epy_block_0, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.epy_block_0, 0), (self.blocks_freqshift_cc_0, 0))


    def get_P_received(self):
        return self.P_received

    def set_P_received(self, P_received):
        self.P_received = P_received

    def get_P_target(self):
        return self.P_target

    def set_P_target(self, P_target):
        self.P_target = P_target

    def get_ch_gain(self):
        return self.ch_gain

    def set_ch_gain(self, ch_gain):
        self.ch_gain = ch_gain
        self.uhd_usrp_sink_0.set_gain(self.ch_gain, 0)

    def get_selector(self):
        return self.selector

    def set_selector(self, selector):
        self.selector = selector

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_sink_0.set_center_freq(self.tx_freq, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_If/self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_TX_ID(self):
        return self.TX_ID

    def set_TX_ID(self, TX_ID):
        self.TX_ID = TX_ID

    def get_F_Of(self):
        return self.F_Of

    def set_F_Of(self, F_Of):
        self.F_Of = F_Of

    def get_F_If(self):
        return self.F_If

    def set_F_If(self, F_If):
        self.F_If = F_If
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_If/self.samp_rate)



def argument_parser():
    description = 'Transmits constant interference for a target noise level. Used to verify the psd logic is correct.'
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--P-received", dest="P_received", type=eng_float, default=eng_notation.num_to_str(float((-44.64))),
        help="Set P_received (float): Measured received power of unit power sinusoid at receiver in dBm. [default=%(default)r]")
    parser.add_argument(
        "--P-target", dest="P_target", type=eng_float, default=eng_notation.num_to_str(float((-126))),
        help="Set         P_target (float): Target power level for normalization in dBm. [default=%(default)r]")
    parser.add_argument(
        "--ch-gain", dest="ch_gain", type=eng_float, default=eng_notation.num_to_str(float(20)),
        help="Set Analog Antenna Gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--selector", dest="selector", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set selector (int): Selects which PSD file to load. 0 = AWGN PSD, 1 = BLE PSD [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Frontend Tx frequency [default=%(default)r]")
    return parser


def main(top_block_cls=psd_calibration_test, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(P_received=options.P_received, P_target=options.P_target, ch_gain=options.ch_gain, selector=options.selector, tx_freq=options.tx_freq)

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
