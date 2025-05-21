#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: BLE_Interference_test
# GNU Radio version: 3.10.1.1

from packaging.version import Version as StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from gnuradio import blocks
import math
import numpy
from gnuradio import digital
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import BLE_Interference_test_epy_block_0 as epy_block_0  # embedded python block



from gnuradio import qtgui

class BLE_Interference_test(gr.top_block, Qt.QWidget):

    def __init__(self, ch_gain=20, estPr=-86.98, targPi=-125, tx_freq=2.4e9):
        gr.top_block.__init__(self, "BLE_Interference_test", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("BLE_Interference_test")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "BLE_Interference_test")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

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
        self.psd_file_path = psd_file_path = "/n/deer/z/samnolan/ww_uhd/host/osla-host/interference/matlab/BLEwaveform/BLE_PSD.csv"
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
        self.epy_block_0 = epy_block_0.blk(sampling_rate=samp_rate, PSD_path=psd_file_path, noise_intensity=200, noise_length=.002, target_Pi=targPi, estimated_Pr=estPr, F_of=F_Of)
        self.digital_gfsk_mod_0_0_0_1 = digital.gfsk_mod(
            samples_per_symbol=round(BLE_sym_length*samp_rate),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.digital_gfsk_mod_0_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=round(BLE_sym_length*samp_rate),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.digital_gfsk_mod_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=round(BLE_sym_length*samp_rate),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.blocks_message_debug_0 = blocks.message_debug(True)
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


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "BLE_Interference_test")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

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

    def get_psd_file_path(self):
        return self.psd_file_path

    def set_psd_file_path(self, psd_file_path):
        self.psd_file_path = psd_file_path

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
        "--estPr", dest="estPr", type=eng_float, default=eng_notation.num_to_str(float(-86.98)),
        help="Set estimated_Pr [default=%(default)r]")
    parser.add_argument(
        "--targPi", dest="targPi", type=eng_float, default=eng_notation.num_to_str(float(-125)),
        help="Set target_Pi [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.4e9)),
        help="Set Analog Antenna TX Frequency [default=%(default)r]")
    return parser


def main(top_block_cls=BLE_Interference_test, options=None):
    if options is None:
        options = argument_parser().parse_args()

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(ch_gain=options.ch_gain, estPr=options.estPr, targPi=options.targPi, tx_freq=options.tx_freq)

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
