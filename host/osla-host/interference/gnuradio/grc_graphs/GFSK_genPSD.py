#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
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
import math
import numpy as np



from gnuradio import qtgui

class GFSK_genPSD(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
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

        self.settings = Qt.QSettings("GNU Radio", "GFSK_genPSD")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10000000
        self.BLE_fd = BLE_fd = 250000
        self.sensitivity = sensitivity = 2*math.pi*BLE_fd/samp_rate
        self.BLE_sym_length = BLE_sym_length = .000001

        ##################################################
        # Blocks
        ##################################################
        self.digital_gfsk_mod_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=round(BLE_sym_length*samp_rate),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.blocks_throttle_0_0_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_streams_to_vector_0 = blocks.streams_to_vector(gr.sizeof_short*1, 2)
        self.blocks_head_0_0 = blocks.head(gr.sizeof_short*2, 20000)
        self.blocks_float_to_short_0_0 = blocks.float_to_short(1, 32767)
        self.blocks_float_to_short_0 = blocks.float_to_short(1, 32767)
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_short*2, '/home/samnolan/OSLA_research/ww_uhd/host/osla-host/interference/gnuradio/log/c16_gfskPSDcsv_10M.bin', False)
        self.blocks_file_sink_0_0.set_unbuffered(False)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_imag_0 = blocks.complex_to_imag(1)
        self.analog_random_source_x_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_random_source_x_0, 0), (self.digital_gfsk_mod_0_0_0, 0))
        self.connect((self.blocks_complex_to_imag_0, 0), (self.blocks_float_to_short_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_float_to_short_0_0, 0))
        self.connect((self.blocks_float_to_short_0, 0), (self.blocks_streams_to_vector_0, 1))
        self.connect((self.blocks_float_to_short_0_0, 0), (self.blocks_streams_to_vector_0, 0))
        self.connect((self.blocks_head_0_0, 0), (self.blocks_file_sink_0_0, 0))
        self.connect((self.blocks_streams_to_vector_0, 0), (self.blocks_head_0_0, 0))
        self.connect((self.blocks_throttle_0_0_0, 0), (self.blocks_complex_to_imag_0, 0))
        self.connect((self.blocks_throttle_0_0_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0, 0), (self.blocks_throttle_0_0_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "GFSK_genPSD")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)
        self.blocks_throttle_0_0_0.set_sample_rate(self.samp_rate)

    def get_BLE_fd(self):
        return self.BLE_fd

    def set_BLE_fd(self, BLE_fd):
        self.BLE_fd = BLE_fd
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)

    def get_sensitivity(self):
        return self.sensitivity

    def set_sensitivity(self, sensitivity):
        self.sensitivity = sensitivity

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length




def main(top_block_cls=GFSK_genPSD, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

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
