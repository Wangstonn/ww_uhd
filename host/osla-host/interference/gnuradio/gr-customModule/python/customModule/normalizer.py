#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2025 Winston Wang.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from gnuradio import gr

import os # To load PSD file
from pathlib import Path

class normalizer(gr.sync_block):
    """
    GNU Radio block to normalize an input complex signal to a target power level
    based on a known power spectral density (PSD) profile.

    This block computes a gain factor that scales the input signal such that
    its power matches a desired target power (in dB), assuming the received power
    is known. The gain is computed using the in-band power extracted from a PSD
    file generated (e.g., via MATLAB's pwelch). The PSD is used to model the spectral
    shape of additive noise or interference.

    Parameters:
        fs (float): Sampling frequency in Hz. Used to scale PSD from digital to analog units.
        P_received (float): Measured received power of the signal in dBm.
        P_target (float): Target power level for normalization in dBm.
        bw (float): Bandwidth over which to compute power, in Hz.
        fc (float): Center frequency of interest within the PSD, in Hz.
        selector (int): Selects which PSD file to load.
                        0 = AWGN PSD (awgn_psd.csv)
                        1 = BLE PSD (ble_psd.csv)

    Raises:
        RuntimeError: If PSD file cannot be found or loaded properly.
        ValueError: If selector is not 0 or 1.
    """
    def __init__(self, fs=10e6, P_received=-44.64, P_target=-126.84, bw=18.5e3, fc=595e3, selector=0):
        gr.sync_block.__init__(
            self,
            name='Normalizer',   # will show up in GRC
            in_sig=[np.complex64],
            out_sig=[np.complex64]
        )
        # Persistent parameters needed during work()
        self.fs = fs
        self.P_received = P_received
        self.P_target = P_target

        #generate normalizer gain from LUT
        match selector:
            case 0:
                PSD_path = os.path.join(os.path.dirname(__file__), "../../../../matlab/psd/awgn_psd.csv") #'C:/Users/wangston/My Drive/OSLA/bpsk/ww_uhd/host/osla-host/interference/matlab/BLEwaveform/gaussian_PSD.csv'
        # PSD_path = 'C:/Users/wangston/My Drive/OSLA/bpsk/ww_uhd/host/osla-host/interference/matlab/psd/awgn_psd.csv'
            case 1:
                PSD_path = os.path.join(os.path.dirname(__file__), "../../../../matlab/psd/ble_psd.csv") #'C:/Users/wangston/My Drive/OSLA/bpsk/ww_uhd/host/osla-host/interference/matlab/BLEwaveform/gaussian_PSD.csv'
            case _:
                raise ValueError("Selector must be 0 or 1, got {}".format(selector))

        PSD = None  # prevent undefined var

        try:
            PSD_file = Path(PSD_path)
            if not PSD_file.is_file():
                raise FileNotFoundError(f"File does not exist: {PSD_file}")
            
            PSD = np.genfromtxt(PSD_file, delimiter=',', dtype=np.double)
            if PSD.ndim < 2 or PSD.shape[1] < 1:
                raise ValueError("PSD file must be at least 2D with one column.")

            print(f"[NoiseController] PSD loaded with shape {PSD.shape}")

        except Exception as e:
            raise RuntimeError(f"[NoiseController ERROR] Failed to load PSD: {e}")

        # Scale signal to match the target received power
        df = 100
        #fs_psd = 10000000
        PSD_i_low  = int(np.round((len(PSD)/2) + (fc/df) - (bw/df)/2) - 1)
        PSD_i_high = int(np.round((len(PSD)/2) + (fc/df) + (bw/df)/2))    

        # in band power = sum(PSD)*df*fs
        P_band = df*np.sum(PSD[PSD_i_low:PSD_i_high,0]*fs)
        self.G = np.sqrt(1/(P_band))


    def work(self, input_items, output_items):
        output_items[0][:] = self.G*10**((self.P_target-self.P_received)/20)*input_items[0]
        return len(output_items[0])
