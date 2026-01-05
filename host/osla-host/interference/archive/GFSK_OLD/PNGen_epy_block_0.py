"""
Author: Sam Nolan

This block reads a Preamble file and then will store it internally
It will then pad the saved preamble with multiple samples according to symbol length
and then repeat indefinitely
"""

import numpy as np
from gnuradio import gr


class blk(gr.basic_block):  # other base classes are basic_block, decim_block, interp_block
    """PN Generator"""

    def __init__(self):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='PN Generator',   # will show up in GRC
            in_sig=None,
            out_sig=[np.float32]
        )
        
        
        self.PN_sequence = np.genfromtxt('/n/deer/z/samnolan/preamble.mem',dtype=np.uint8, unpack=True)

        self.buff = 0


    def work(self, input_items, output_items):
        out = output_items[0]
        for i in range(len(out)):
            if (self.buff + i) < len(self.PN_sequence):
                out[i] = 2*self.PN_sequence[self.buff + i] - 1
            else:
                self.buff = -i
                out[i] = 2*self.PN_sequence[self.buff + i] - 1
        
        self.buff = self.buff + len(out)

        return len(out) 
