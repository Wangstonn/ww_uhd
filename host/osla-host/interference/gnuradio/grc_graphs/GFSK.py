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

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import analog
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
import GFSK_epy_block_0_0 as epy_block_0_0  # embedded python block
import numpy as np



from gnuradio import qtgui

class GFSK(gr.top_block, Qt.QWidget):

    def __init__(self):
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

        self.settings = Qt.QSettings("GNU Radio", "GFSK")

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
        self.samp_rate_0 = samp_rate_0 = 10000000
        self.Pr = Pr = -93.38235
        self.F_band = F_band = 2*200000000/336/32
        self.F_Of = F_Of = 0
        self.F_IF = F_IF = 595238
        self.Es_Ni = Es_Ni = -5
        self.BLE_sym_length = BLE_sym_length = .000001

        ##################################################
        # Blocks
        ##################################################
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            1024, #size
            samp_rate, #samp_rate
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.epy_block_0_0 = epy_block_0_0.blk(sampling_rate=samp_rate, noise_rate=250, noise_length=.002, Es_Ni=Es_Ni, Pr=Pr, F_of=-F_Of)
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
        self.blocks_throttle_0_0_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_streams_to_vector_0 = blocks.streams_to_vector(gr.sizeof_short*1, 2)
        self.blocks_null_source_0 = blocks.null_source(gr.sizeof_gr_complex*1)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_multiply_const_xx_0 = blocks.multiply_const_cc(10**((Pr+137)/20), 1)
        self.blocks_message_debug_0 = blocks.message_debug(True)
        self.blocks_head_0_0 = blocks.head(gr.sizeof_short*2, 20000000)
        self.blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, 20000000)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*-F_IF/samp_rate)
        self.blocks_float_to_short_0_0 = blocks.float_to_short(1, 1)
        self.blocks_float_to_short_0 = blocks.float_to_short(1, 1)
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_short*2, '/home/samnolan/OSLA_research/ww_uhd/host/osla-host/interference/gnuradio/log/c16_sim_gfsk_10M.bin', False)
        self.blocks_file_sink_0_0.set_unbuffered(False)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_imag_0 = blocks.complex_to_imag(1)
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.analog_random_source_x_0_1 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)
        self.analog_random_source_x_0_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)
        self.analog_random_source_x_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 2000))), True)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, 7.5, 0)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.epy_block_0_0, 'Debug'), (self.blocks_message_debug_0, 'print'))
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.analog_random_source_x_0, 0), (self.digital_gfsk_mod_0_0_0, 0))
        self.connect((self.analog_random_source_x_0_0, 0), (self.digital_gfsk_mod_0_0_0_0, 0))
        self.connect((self.analog_random_source_x_0_1, 0), (self.digital_gfsk_mod_0_0_0_1, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.blocks_complex_to_imag_0, 0), (self.blocks_float_to_short_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_float_to_short_0_0, 0))
        self.connect((self.blocks_float_to_short_0, 0), (self.blocks_streams_to_vector_0, 1))
        self.connect((self.blocks_float_to_short_0_0, 0), (self.blocks_streams_to_vector_0, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.blocks_head_0, 0))
        self.connect((self.blocks_head_0, 0), (self.blocks_multiply_const_xx_0, 0))
        self.connect((self.blocks_head_0_0, 0), (self.blocks_file_sink_0_0, 0))
        self.connect((self.blocks_multiply_const_xx_0, 0), (self.blocks_throttle_0_0_0, 0))
        self.connect((self.blocks_null_source_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.blocks_streams_to_vector_0, 0), (self.blocks_head_0_0, 0))
        self.connect((self.blocks_throttle_0_0_0, 0), (self.blocks_complex_to_imag_0, 0))
        self.connect((self.blocks_throttle_0_0_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0, 0), (self.epy_block_0_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0_0, 0), (self.epy_block_0_0, 1))
        self.connect((self.digital_gfsk_mod_0_0_0_1, 0), (self.epy_block_0_0, 2))
        self.connect((self.epy_block_0_0, 0), (self.blocks_freqshift_cc_0, 0))
        self.connect((self.epy_block_0_0, 0), (self.qtgui_time_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "GFSK")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*-self.F_IF/self.samp_rate)
        self.blocks_throttle_0_0_0.set_sample_rate(self.samp_rate)
        self.epy_block_0_0.sampling_rate = self.samp_rate
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)

    def get_BLE_fd(self):
        return self.BLE_fd

    def set_BLE_fd(self, BLE_fd):
        self.BLE_fd = BLE_fd
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.samp_rate)

    def get_sensitivity(self):
        return self.sensitivity

    def set_sensitivity(self, sensitivity):
        self.sensitivity = sensitivity

    def get_samp_rate_0(self):
        return self.samp_rate_0

    def set_samp_rate_0(self, samp_rate_0):
        self.samp_rate_0 = samp_rate_0

    def get_Pr(self):
        return self.Pr

    def set_Pr(self, Pr):
        self.Pr = Pr
        self.blocks_multiply_const_xx_0.set_k(10**((self.Pr+137)/20))
        self.epy_block_0_0.Pr = self.Pr

    def get_F_band(self):
        return self.F_band

    def set_F_band(self, F_band):
        self.F_band = F_band

    def get_F_Of(self):
        return self.F_Of

    def set_F_Of(self, F_Of):
        self.F_Of = F_Of

    def get_F_IF(self):
        return self.F_IF

    def set_F_IF(self, F_IF):
        self.F_IF = F_IF
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*-self.F_IF/self.samp_rate)

    def get_Es_Ni(self):
        return self.Es_Ni

    def set_Es_Ni(self, Es_Ni):
        self.Es_Ni = Es_Ni

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length




def main(top_block_cls=GFSK, options=None):

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
