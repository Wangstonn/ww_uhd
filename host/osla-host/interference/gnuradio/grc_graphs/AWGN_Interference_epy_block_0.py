"""
Author: Samuel Nolan
Description:
Takes in three noise sources on the input and randomly adds them together
according to a poisson distribution. Internally normalizes inband noise power
from settings and then multiplies the amplitude according to a log normal distribution.
"""

import numpy as np
from gnuradio import gr
import pmt
import threading  


import os
from pathlib import Path

class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block Noise Controller"""

    def __init__(self, fs = 10e6, poisson_intensity = 200, pkt_len = 2e-3, P_target = -40, P_received = -72.52, fc = 0):  # only default arguments here
        """
        Parameters:
        sampling rate (Hz): Needed to calculate length of noise frame and wait frame
        poisson_intensity (arrivals/second): noise packet rate /s (can overlap up to 3 packets)
        noise length (seconds): length of interference packet (if too large may just overlap constantly)
        Es_Ni(dBm): Desired Es/Ni (Es is hard coded internally)
        Pr(dB): estimate of received power from sending a digital normalized signal 
        fc (Hz): offset of OSLA from center of BLE noise (-+). Used in in-band normalization calculation.
        analog_gain (dB): To prevent saturation, set analog gain accordingly
        """
        self._lock = threading.Lock() 
        gr.sync_block.__init__(
            self,
            name='Noise Controller',   # will show up in GRC
            in_sig=[np.complex64,np.complex64,np.complex64],
            out_sig=[np.complex64]
        )
        
        #n_counters[0]: state of interferer
        #n_counters[1]: noise clk(how long its been on)
        #n_counters[2]: gain of interferer (decided on log normal distribution)
        #n_counters[3]: last n_samples of a frame that should include noise
        self.n_counters = np.array([[False,0,1.0,0], [False,0,1.0,0], [False,0,1.0,0]])
        # Persistent parameters needed during work()
        self.theta = np.random.uniform(size=len(self.n_counters))*2j*np.pi #Initialize the phase offset for each interferer

        self.fs = fs
        self.P_received = P_received
        self.P_target = P_target
        self.poisson_intensity = poisson_intensity
        self.pkt_len = pkt_len
        self.lognormVar = 0#10 #variance of log normal distribution

        self.noise_frame = np.round((fs*self.pkt_len),0)

        self.update_params(False, P_target, P_received)

        self.arrival_clk = 0
        self.wait_frame = round(((-1/self.poisson_intensity)*np.log(np.random.uniform())*self.fs))
        
        self.idx = 0
        self.full = False
        
        #generate normalizer gain from LUT
        #run locally from grc_graphs folder
        #PSD_path =  os.path.join(os.path.dirname(__file__), "../../matlab/psd/awgn_psd.csv") #'C:/Users/wangston/My Drive/OSLA/bpsk/ww_uhd/host/osla-host/interference/matlab/BLEwaveform/gaussian_PSD.csv'
        PSD_path = 'C:/Users/wangston/My Drive/OSLA/bpsk/ww_uhd/host/osla-host/interference/matlab/psd/awgn_psd.csv'
        print(f"[NoiseController] Loading hardcoded PSD file: {PSD_path}")

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

        #Bandwidth of OSLA signal hardcoded here
        #336 samples/chip, 32 chips/symbol, 200000000 samples/second
        Ts = 336*32/200000000
        self.BW = 2/Ts
        df = 100 #resolution of psd file
        #fs_psd = 10000000 #sampling rate used in psd file
        PSD_i_low  = int(np.round((len(PSD)/2) + (fc/df) - (self.BW/df)/2) - 1)
        PSD_i_high = int(np.round((len(PSD)/2) + (fc/df) + (self.BW/df)/2))    

        #in band power = sum(PSD)*df*fs
        P_band = df*np.sum(PSD[PSD_i_low:PSD_i_high,0]*fs)
        self.G = np.sqrt(1/(P_band))
        print("Gain: ", self.G*10**((self.P_target-self.P_received)/20))

        self.DebugPortName = 'Debug'
        self.message_port_register_out(pmt.intern(self.DebugPortName))
    
    #Update packet generation parameters using the input parameters    
    def update_params(self, enabled, P_target, P_received):
        with self._lock:
            self.enabled = enabled
            mu_linear = 1/(self.poisson_intensity*self.pkt_len)*np.exp((self.lognormVar*(np.log(10))**2)/200) #TODO: this should be division
            self.mu = P_target + 10*np.log10(mu_linear)
            self.Pr = P_received

    def work(self, input_items, output_items):
        with self._lock:
            enabled = self.enabled
            mu = self.mu
            Pr = self.Pr
        # use 'enabled', `mu` and `Pr` instead of self.mu/self.Pr in the rest of the function
        
        #gate activity of block using enabled flag
        if not enabled:
            # Work output the number of output items produced. Return 0 to pause downstream blocks.
            return 0

        self.arrival_clk = self.arrival_clk + len(output_items[0])
        self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
        if (self.idx == None) and (self.arrival_clk >= self.wait_frame):
            self.full = True
        '''
        #use this to send messages from the debug port
            PMT_msg = pmt.from_bool(self.state)
            self.message_port_pub(pmt.intern(self.DebugPortName), PMT_msg)
        '''

        #run arrival clk and signal when packet should start transmitting
        if ((self.arrival_clk >= self.wait_frame) and not self.full):
            self.arrival_clk = self.arrival_clk - self.wait_frame
            self.wait_frame = round(((-1/self.poisson_intensity)*np.log(np.random.uniform())*self.fs))
            #Mark Interferer to start
            self.n_counters[self.idx][0] = True
            #generate gain value from parameters
            P = np.random.normal(loc=mu, scale=self.lognormVar)
            self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-Pr)/10))
            #generate new phase offset
            self.theta[self.idx] = np.random.uniform()*2j*np.pi
            self.n_counters[self.idx][3] = self.arrival_clk
            
            
            #if next wait_frame occurs in same window
            self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
            if ((self.arrival_clk >= self.wait_frame) and (self.idx != None)):
                self.arrival_clk = self.arrival_clk - self.wait_frame
                self.wait_frame = round(((-1/self.poisson_intensity)*np.log(np.random.uniform())*self.fs))
                #Mark Interferer to start
                self.n_counters[self.idx][0] = True
                #generate gain value from parameters
                P = np.random.normal(loc=mu, scale=self.lognormVar)
                self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-Pr)/10))
                #generate new phase offset
                self.theta[self.idx] = np.random.uniform()*2j*np.pi
                self.n_counters[self.idx][3] = self.arrival_clk

                #if all three interferers occur in the same window
                self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
                if ((self.arrival_clk >= self.wait_frame) and (self.idx != None)):
                    self.arrival_clk = self.arrival_clk - self.wait_frame
                    self.wait_frame = round(((-1/self.poisson_intensity)*np.log(np.random.uniform())*self.fs))
                    #Mark Interferer to start
                    self.n_counters[self.idx][0] = True
                    #generate gain value from parameters
                    P = np.random.normal(loc=mu, scale=self.lognormVar)
                    self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-Pr)/10))
                    #generate new phase offset
                    self.theta[self.idx] = np.random.uniform()*2j*np.pi
                    self.n_counters[self.idx][3] = self.arrival_clk
            
            #PMT_msg = pmt.to_pmt(self.wait_frame)
            #self.message_port_pub(pmt.intern(self.DebugPortName), PMT_msg)

        #If we are full we want to start noise immediately and also reset the arrival clock to prevent error buildup
        if ((self.idx != None) and self.full):
            self.arrival_clk = len(output_items[0])
            self.wait_frame = round(((-1/self.poisson_intensity)*np.log(np.random.uniform())*self.fs))
            #Mark Interferer to start
            self.n_counters[self.idx][0] = True
            #generate gain value from parameters
            P = np.random.normal(loc=mu, scale=self.lognormVar)
            self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-Pr)/10))
            #generate new phase offset
            self.theta[self.idx] = np.random.uniform()*2j*np.pi
            self.full = False

            
        output_items[0][:] = 0
        for i in range(len(self.n_counters)):
            if(self.n_counters[i][0]):

                if(self.n_counters[i][3] != 0):
                    self.n_counters[i][1] = self.n_counters[i][1] + self.n_counters[i][3]
                    output_items[0][-int(self.n_counters[i][3]):] = output_items[0][-int(self.n_counters[i][3]):] + self.n_counters[i][2]*input_items[i][-int(self.n_counters[i][3]):]*np.exp(self.theta[i])
                    self.n_counters[i][3] = 0
                else:
                    self.n_counters[i][1] = self.n_counters[i][1] + len(output_items[0])
                    
                    if(self.n_counters[i][1] > self.noise_frame):
                        end_n_elem = len(output_items[0]) - (self.n_counters[i][1] - self.noise_frame)
                        output_items[0][:int(end_n_elem)] = output_items[0][:int(end_n_elem)] + self.n_counters[i][2]*input_items[i][:int(end_n_elem)]*np.exp(self.theta[i])
                        self.n_counters[i][0] = False
                        self.n_counters[i][1] = 0
                    else:
                        output_items[0][:] = output_items[0][:] + self.n_counters[i][2]*input_items[i]*np.exp(self.theta[i])

                

        if(abs(output_items[0][0]) > 1):
            PMT_msg = pmt.string_to_symbol("Warning: USRP is saturated, please edit analog gain in flowgraph")
            self.message_port_pub(pmt.intern(self.DebugPortName), PMT_msg)
            #return -1
        return len(output_items[0][:])
