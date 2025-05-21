#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: BLE_Interference
# GNU Radio version: 3.10.12.0

from gnuradio import blocks
import math
import numpy
from gnuradio import blocks, gr
from gnuradio import digital
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
import BLE_Interference_epy_block_0 as epy_block_0  # embedded python block
import threading

import os
import socket
import ctypes
import selectors

# For socket communication
SERVER_PORT = 12345
class MSG_t(ctypes.Structure):
  _fields_ = [("event", ctypes.c_bool),
              ("intf_rss_dbm", ctypes.c_double),
              ("target_intf_rss_dbm", ctypes.c_double),
              ]
  def __init__(self):
    self.event: ctypes.c_bool = False
    self.intf_rss_dbm: ctypes.c_double = 0
    self.target_intf_rss_dbm: ctypes.c_double = 0
  def __str__(self):
     return f"""MSG_t
event: {self.event}
intf_rss_dbm: {self.intf_rss_dbm}
target_intf_rss_dbm: {self.target_intf_rss_dbm}
"""


class BLE_Interference(gr.top_block):

    def __init__(self, ch_gain=20, estPr=(-86.98), targPi=(-125), tx_freq=2.4e9):
        gr.top_block.__init__(self, "BLE_Interference", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.ch_gain = ch_gain
        self.estPr = estPr
        self.targPi = targPi
        self.tx_freq = tx_freq

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.BLE_fd = BLE_fd = 250000
        self.sensitivity = sensitivity = 2*math.pi*BLE_fd/samp_rate
        self.TX_ID = TX_ID = "addr=192.168.10.2"
        self.F_Of = F_Of = 0
        self.F_IF = F_IF = 595238
        self.BLE_sym_length = BLE_sym_length = .000001

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
        self.epy_block_0 = epy_block_0.blk(sampling_rate=samp_rate, noise_intensity=200, noise_length=.002, target_Pi=targPi, estimated_Pr=estPr, F_of=F_Of)
        self.digital_gfsk_mod_0_0_0_1 = digital.gfsk_mod(
            samples_per_symbol=(round(BLE_sym_length*samp_rate)),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.digital_gfsk_mod_0_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=(round(BLE_sym_length*samp_rate)),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.digital_gfsk_mod_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=(round(BLE_sym_length*samp_rate)),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.blocks_message_debug_0 = blocks.message_debug(True, gr.log_levels.info)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_IF/samp_rate)
        self.analog_random_source_x_0_1 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 100000))), True)
        self.analog_random_source_x_0_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 100000))), True)
        self.analog_random_source_x_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 100000))), True)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.epy_block_0, 'Debug'), (self.blocks_message_debug_0, 'print'))
        self.connect((self.analog_random_source_x_0, 0), (self.digital_gfsk_mod_0_0_0, 0))
        self.connect((self.analog_random_source_x_0_0, 0), (self.digital_gfsk_mod_0_0_0_0, 0))
        self.connect((self.analog_random_source_x_0_1, 0), (self.digital_gfsk_mod_0_0_0_1, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0, 0), (self.epy_block_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0_0, 0), (self.epy_block_0, 1))
        self.connect((self.digital_gfsk_mod_0_0_0_1, 0), (self.epy_block_0, 2))
        self.connect((self.epy_block_0, 0), (self.blocks_freqshift_cc_0, 0))


    def get_ch_gain(self):
        return self.ch_gain

    def set_ch_gain(self, ch_gain):
        self.ch_gain = ch_gain
        self.uhd_usrp_sink_0.set_gain(self.ch_gain, 0)

    def get_estPr(self):
        return self.estPr

    def set_estPr(self, estPr):
        self.estPr = estPr

    def get_targPi(self):
        return self.targPi

    def set_targPi(self, targPi):
        self.targPi = targPi

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_sink_0.set_center_freq(self.tx_freq, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.samp_rate)
        self.epy_block_0.sampling_rate = self.samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_BLE_fd(self):
        return self.BLE_fd

    def set_BLE_fd(self, BLE_fd):
        self.BLE_fd = BLE_fd
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)

    def get_sensitivity(self):
        return self.sensitivity

    def set_sensitivity(self, sensitivity):
        self.sensitivity = sensitivity

    def get_TX_ID(self):
        return self.TX_ID

    def set_TX_ID(self, TX_ID):
        self.TX_ID = TX_ID

    def get_F_Of(self):
        return self.F_Of

    def set_F_Of(self, F_Of):
        self.F_Of = F_Of

    def get_F_IF(self):
        return self.F_IF

    def set_F_IF(self, F_IF):
        self.F_IF = F_IF
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.samp_rate)

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--ch-gain", dest="ch_gain", type=eng_float, default=eng_notation.num_to_str(float(20)),
        help="Set Analog Antenna Gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--estPr", dest="estPr", type=eng_float, default=eng_notation.num_to_str(float((-86.98))),
        help="Set estimated_Pr [default=%(default)r]")
    parser.add_argument(
        "--targPi", dest="targPi", type=eng_float, default=eng_notation.num_to_str(float((-125))),
        help="Set target_Pi [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Frontend Tx frequency [default=%(default)r]")
    return parser


def main(top_block_cls=BLE_Interference, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(ch_gain=options.ch_gain, estPr=options.estPr, targPi=options.targPi, tx_freq=options.tx_freq)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set() 
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('141.213.15.85', SERVER_PORT))
    server_socket.listen(1)
    print(f"Server is listening on port {SERVER_PORT}...")

    # Use selectors to listen for connections
    sel = selectors.DefaultSelector()
    sel.register(server_socket, selectors.EVENT_READ)
    
    while True:
        for key, _ in sel.select():
            if key.fileobj == server_socket:
                client_socket, addr = server_socket.accept()
                print(f"Connection from {addr} has been established.")
                sel.register(client_socket, selectors.EVENT_READ)
            else:
                client_socket = key.fileobj
                data = client_socket.recv(ctypes.sizeof(MSG_t))
                if data:
                    msg = MSG_t.from_buffer_copy(data)
                    print(f"Received data: {msg}")
                    tb.epy_block_0.update_params(msg.event, msg.target_intf_rss_dbm, msg.intf_rss_dbm)
                else:
                    print(f"Closing connection to {client_socket.getpeername()}")
                    sel.unregister(client_socket)
                    client_socket.close()

    tb.wait()


if __name__ == '__main__':
    main()
