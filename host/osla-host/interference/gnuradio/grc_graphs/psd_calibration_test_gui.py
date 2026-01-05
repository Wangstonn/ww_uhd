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

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import analog
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
import psd_calibration_test_gui_epy_block_0 as epy_block_0  # embedded python block
import sip
import threading



class psd_calibration_test_gui(gr.top_block, Qt.QWidget):

    def __init__(self, P_received=(-41.64), P_target=(-126), ch_gain=20, selector=1, tx_freq=2.2e9):
        gr.top_block.__init__(self, "psd_calibration_test", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("psd_calibration_test")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "psd_calibration_test_gui")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
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
        self.fs = fs = 10000000
        self.BLE_fd = BLE_fd = 250000
        self.sensitivity = sensitivity = 2*math.pi*BLE_fd/fs
        self.TX_ID = TX_ID = "addr=192.168.10.2"
        self.F_Of = F_Of = 0
        self.F_If = F_If = 595238
        self.BLE_sym_length = BLE_sym_length = .000001

        ##################################################
        # Blocks
        ##################################################

        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            1024, #size
            fs, #samp_rate
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
        self.epy_block_0 = epy_block_0.blk(fs=10000000.0, P_received=-44.64, P_target=0, bw=18500.0, fc=F_If + F_Of, selector=selector)
        self.digital_gfsk_mod_0_0_0 = digital.gfsk_mod(
            samples_per_symbol=(round(BLE_sym_length*fs)),
            sensitivity=sensitivity,
            bt=0.5,
            verbose=False,
            log=False,
            do_unpack=False)
        self.blocks_selector_0 = blocks.selector(gr.sizeof_gr_complex*1,selector,0)
        self.blocks_selector_0.set_enabled(True)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*F_If/fs)
        self.analog_random_source_x_0 = blocks.vector_source_b(list(map(int, numpy.random.randint(0, 2, 100000))), True)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, 1, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.blocks_selector_0, 0))
        self.connect((self.analog_random_source_x_0, 0), (self.digital_gfsk_mod_0_0_0, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.blocks_selector_0, 0), (self.epy_block_0, 0))
        self.connect((self.digital_gfsk_mod_0_0_0, 0), (self.blocks_selector_0, 1))
        self.connect((self.epy_block_0, 0), (self.blocks_freqshift_cc_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "psd_calibration_test_gui")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

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

    def get_selector(self):
        return self.selector

    def set_selector(self, selector):
        self.selector = selector
        self.blocks_selector_0.set_input_index(self.selector)

    def get_tx_freq(self):
        return self.tx_freq

    def set_tx_freq(self, tx_freq):
        self.tx_freq = tx_freq

    def get_fs(self):
        return self.fs

    def set_fs(self, fs):
        self.fs = fs
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.fs)
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_If/self.fs)
        self.qtgui_time_sink_x_0.set_samp_rate(self.fs)

    def get_BLE_fd(self):
        return self.BLE_fd

    def set_BLE_fd(self, BLE_fd):
        self.BLE_fd = BLE_fd
        self.set_sensitivity(2*math.pi*self.BLE_fd/self.fs)

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

    def get_F_If(self):
        return self.F_If

    def set_F_If(self, F_If):
        self.F_If = F_If
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.F_If/self.fs)

    def get_BLE_sym_length(self):
        return self.BLE_sym_length

    def set_BLE_sym_length(self, BLE_sym_length):
        self.BLE_sym_length = BLE_sym_length



def argument_parser():
    description = 'Transmits constant interference for a target noise level. Used to verify the psd logic is correct.'
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--P-received", dest="P_received", type=eng_float, default=eng_notation.num_to_str(float((-41.64))),
        help="Set P_received (float): Measured received power of unit power sinusoid at receiver in dBm. [default=%(default)r]")
    parser.add_argument(
        "--P-target", dest="P_target", type=eng_float, default=eng_notation.num_to_str(float((-126))),
        help="Set         P_target (float): Target power level for normalization in dBm. [default=%(default)r]")
    parser.add_argument(
        "--ch-gain", dest="ch_gain", type=eng_float, default=eng_notation.num_to_str(float(20)),
        help="Set Analog Antenna Gain (dB) [default=%(default)r]")
    parser.add_argument(
        "--selector", dest="selector", type=intx, default=1,
        help="Set selector (int): Selects which PSD file to load. 0 = AWGN PSD, 1 = BLE PSD [default=%(default)r]")
    parser.add_argument(
        "--tx-freq", dest="tx_freq", type=eng_float, default=eng_notation.num_to_str(float(2.2e9)),
        help="Set Analog Frontend Tx frequency [default=%(default)r]")
    return parser


def main(top_block_cls=psd_calibration_test_gui, options=None):
    if options is None:
        options = argument_parser().parse_args()

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(P_received=options.P_received, P_target=options.P_target, ch_gain=options.ch_gain, selector=options.selector, tx_freq=options.tx_freq)

    tb.start()
    tb.flowgraph_started.set()

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
