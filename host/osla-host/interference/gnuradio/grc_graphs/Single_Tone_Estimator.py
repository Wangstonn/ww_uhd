#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Single_Tone_Estimator
# Author: Samuel Nolan
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

from gnuradio import analog
from gnuradio import blocks
import math
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



from gnuradio import qtgui

class Single_Tone_Estimator(gr.top_block, Qt.QWidget):

    def __init__(self, tx_freq=2.2e9):
        gr.top_block.__init__(self, "Single_Tone_Estimator", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Single_Tone_Estimator")
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

        self.settings = Qt.QSettings("GNU Radio", "Single_Tone_Estimator")

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
        self.tx_freq = tx_freq

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.TX_ID = TX_ID = "addr=192.168.110.2"
        self.F_IF = F_IF = 595238
        self.CH_gain = CH_gain = 20

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
        self.uhd_usrp_sink_0.set_gain(CH_gain, 0)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_IF/samp_rate)
        self.analog_const_source_x_0 = analog.sig_source_c(0, analog.GR_CONST_WAVE, 0, 0, 1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_freqshift_cc_0, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.uhd_usrp_sink_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "Single_Tone_Estimator")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq
        self.uhd_usrp_sink_0.set_center_freq(self.tx_freq, 0)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_TX_ID(self):
        return self.TX_ID

    def set_TX_ID(self, TX_ID):
        self.TX_ID = TX_ID

    def get_F_IF(self):
        return self.F_IF

    def set_F_IF(self, F_IF):
        self.F_IF = F_IF
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF/self.samp_rate)

    def get_CH_gain(self):
        return self.CH_gain

    def set_CH_gain(self, CH_gain):
        self.CH_gain = CH_gain
        self.uhd_usrp_sink_0.set_gain(self.CH_gain, 0)



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Front end Tx frequency [default=%(default)r]")
    return parser


def main(top_block_cls=Single_Tone_Estimator, options=None):
    if options is None:
        options = argument_parser().parse_args()

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(tx_freq=options.tx_freq)

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
