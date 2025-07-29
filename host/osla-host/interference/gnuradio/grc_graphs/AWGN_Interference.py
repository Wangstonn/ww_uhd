#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: AWGN_Interference
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
import AWGN_Interference_epy_block_0 as epy_block_0  # embedded python block
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


class AWGN_Interference(gr.top_block):

    def __init__(self, ch_gain=0, P_target = -40, P_received = -72.52, tx_freq=2.2e9, use_Socket=1):
        gr.top_block.__init__(self, "AWGN_Interference", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Parameters
        ##################################################
        self.P_received = P_received
        self.P_target = P_target
        self.ch_gain = ch_gain
        self.tx_freq = tx_freq
        self.use_Socket = use_Socket

        ##################################################
        # Variables
        ##################################################
        self.fs = fs = 10000000
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
        self.uhd_usrp_sink_0.set_clock_source('internal', 0)
        self.uhd_usrp_sink_0.set_samp_rate(fs)
        # No synchronization enforced.

        self.uhd_usrp_sink_0.set_center_freq(tx_freq, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_bandwidth(160000000, 0)
        self.uhd_usrp_sink_0.set_gain(ch_gain, 0)
        self.epy_block_0 = epy_block_0.blk(fs=fs, poisson_intensity=200, pkt_len=2e-3, P_target=P_target, P_received=P_received, fc=F_Of)
        self.blocks_message_debug_0 = blocks.message_debug(True, gr.log_levels.info)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_IF/fs)
        self.analog_noise_source_x_0_1 = analog.noise_source_c(analog.GR_GAUSSIAN, 1, 2)
        self.analog_noise_source_x_0_0 = analog.noise_source_c(analog.GR_GAUSSIAN, 1, 1)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, 1, 0)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.epy_block_0, 'Debug'), (self.blocks_message_debug_0, 'print'))
        self.connect((self.analog_noise_source_x_0, 0), (self.epy_block_0, 0))
        self.connect((self.analog_noise_source_x_0_0, 0), (self.epy_block_0, 1))
        self.connect((self.analog_noise_source_x_0_1, 0), (self.epy_block_0, 2))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.epy_block_0, 0), (self.blocks_freqshift_cc_0, 0))


    def get_P_received(self):
        return self.P_received

    def set_P_received(self, P_received):
        self.P_received = P_received
        self.epy_block_0.P_received = self.P_received

    def get_P_target(self):
        return self.P_target

    def set_P_target(self, P_target):
        self.P_target = P_target
        self.epy_block_0.P_target = self.P_target

    def get_ch_gain(self):
        return self.ch_gain

    def set_ch_gain(self, ch_gain):
        self.ch_gain = ch_gain
        self.uhd_usrp_sink_0.set_gain(self.ch_gain, 0)

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_sink_0.set_center_freq(self.tx_freq, 0)

    def get_use_Socket(self):
        return self.use_Socket

    def set_use_Socket(self, use_Socket):
        self.use_Socket = use_Socket

    def get_fs(self):
        return self.fs

    def set_fs(self, fs):
        self.fs = fs
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.fs)
        self.epy_block_0.fs = self.fs
        self.uhd_usrp_sink_0.set_samp_rate(self.fs)

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
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.fs)

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--P-received", dest="P_received", type=eng_float, default=eng_notation.num_to_str(float((-72.52))),
        help="Set P_received [default=%(default)r]")
    parser.add_argument(
        "--P-target", dest="P_target", type=eng_float, default=eng_notation.num_to_str(float((-40))),
        help="Set P_target [default=%(default)r]")
    parser.add_argument(
        "--ch-gain", dest="ch_gain", type=eng_float, default=eng_notation.num_to_str(float(0)),
        help="Set Analog Antenna Gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Frontend Tx frequency [default=%(default)r]")
    parser.add_argument(
        "--use-Socket", dest="use_Socket", type=intx, default=1,
        help="Set 1 = use socket. 0 = use pr and pi set by input [default=%(default)r]")
    return parser


def main(top_block_cls=AWGN_Interference, options=None):
    if options is None:
        options = argument_parser().parse_args()
    tb = top_block_cls(P_received=options.P_received, P_target=options.P_target, ch_gain=options.ch_gain, tx_freq=options.tx_freq, use_Socket=options.use_Socket)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()
    if tb.use_Socket:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', SERVER_PORT)) #141.213.15.85 AA4.eecs.umich.edu
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
    else:
        tb.flowgraph_started.wait()
        print("Activating module without socket...")
        tb.epy_block_0.update_params(True, tb.P_target, tb.P_received)

    tb.wait()


if __name__ == '__main__':
    main()
