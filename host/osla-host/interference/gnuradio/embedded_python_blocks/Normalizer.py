"""
Embedded Python Blocks:

This block is a stand-alone block that takes in 1 BLE interferer to test 
normalizing in band noise capabilities

"""

import numpy as np
from gnuradio import gr


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Normalizer"""

    def __init__(self, sampling_rate = 32000, F_band = 30000, F_if = 0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        
        gr.sync_block.__init__(
            self,
            name='Normalizer',   # will show up in GRC
            in_sig=[np.complex64],
            out_sig=[np.complex64]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.sampling_rate = sampling_rate
        self.F_if = F_if
        self.F_band = F_band

        #generate normalizer gain from LUT
        file_path = '/home/samnolan/OSLA_research/OSLA/bpsk/interference/BLEwaveform/BLE_PSD.csv'

        try:
            PSD = np.genfromtxt(file_path,delimiter=',', dtype=np.double)
        except FileNotFoundError:
            print(f"Error: File not found at {file_path}")

        fbin_width = 100

        self.G = np.sqrt(1/(fbin_width*np.sum(PSD[(len(PSD)//2) - 1 + (F_if//fbin_width) - (F_band//fbin_width)//2:(len(PSD)//2) - 1 + (F_if//fbin_width) + (F_band//fbin_width)//2,0])))

    def work(self, input_items, output_items):
        output_items[0][:] = self.G*input_items[0]

        return len(output_items[0])
