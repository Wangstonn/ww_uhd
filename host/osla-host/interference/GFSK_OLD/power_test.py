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

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import blocks
import math
import numpy
from gnuradio import digital
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import power_test_epy_block_0 as epy_block_0  # embedded python block



from gnuradio import qtgui

class power_test(gr.top_block, Qt.QWidget):

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

        self.settings = Qt.QSettings("GNU Radio", "power_test")

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
        self.samp_rate = samp_rate = 16000000
        self.fd = fd = 250000
        self.sensitivity = sensitivity = 2*math.pi*fd/samp_rate
        self.length = length = 336*32
        self.Pr = Pr = -90
        self.F_band = F_band = 2*200000000/32/336
        self.F_Of = F_Of = 0
        self.F_IF = F_IF = -595238
        self.Es_Ni = Es_Ni = 0
        self.BLE_sym_length = BLE_sym_length = .000001

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0_0 = uhd.usrp_source(
            ",".join(("addr=192.168.10.2", '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0_0.set_time_unknown_pps(uhd.time_spec(0))

        self.uhd_usrp_source_0_0.set_center_freq(2400000000, 0)
        self.uhd_usrp_source_0_0.set_antenna("RX2", 0)
        self.uhd_usrp_source_0_0.set_bandwidth(160000000, 0)
        self.uhd_usrp_source_0_0.set_gain(0, 0)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join((" addr=192.168.10.2", '')),
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

        self.uhd_usrp_sink_0.set_center_freq(2400000000, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_sink_0.set_bandwidth(160000000, 0)
        self.uhd_usrp_sink_0.set_gain(20, 0)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            32768, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.epy_block_0 = epy_block_0.blk(sampling_rate=samp_rate, noise_rate=200, noise_length=.002, Es_Ni=Es_Ni, Pr=Pr, F_of=-F_Of)
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
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_IF + F_Of/samp_rate)
        self.analog_random_source_x_0_1 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)
        self.analog_random_source_x_0_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)
        self.analog_random_source_x_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)


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
        self.connect((self.uhd_usrp_source_0_0, 0), (self.qtgui_freq_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "power_test")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_sensitivity(2*math.pi*self.fd/self.samp_rate)
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF + self.F_Of/self.samp_rate)
        self.epy_block_0.sampling_rate = self.samp_rate
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_0_0.set_samp_rate(self.samp_rate)

    def get_fd(self):
        return self.fd

    def set_fd(self, fd):
        self.fd = fd
        self.set_sensitivity(2*math.pi*self.fd/self.samp_rate)

    def get_sensitivity(self):
        return self.sensitivity

    def set_sensitivity(self, sensitivity):
        self.sensitivity = sensitivity

    def get_length(self):
        return self.length

    def set_length(self, length):
        self.length = length

    def get_Pr(self):
        return self.Pr

    def set_Pr(self, Pr):
        self.Pr = Pr
        self.epy_block_0.Pr = self.Pr

    def get_F_band(self):
        return self.F_band

    def set_F_band(self, F_band):
        self.F_band = F_band

    def get_F_Of(self):
        return self.F_Of

    def set_F_Of(self, F_Of):
        self.F_Of = F_Of
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF + self.F_Of/self.samp_rate)

    def get_F_IF(self):
        return self.F_IF

    def set_F_IF(self, F_IF):
        self.F_IF = F_IF
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_IF + self.F_Of/self.samp_rate)

    def get_Es_Ni(self):
        return self.Es_Ni

    def set_Es_Ni(self, Es_Ni):
        self.Es_Ni = Es_Ni

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length




def main(top_block_cls=power_test, options=None):

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
