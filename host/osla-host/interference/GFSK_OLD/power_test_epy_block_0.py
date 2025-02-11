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


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block Noise Controller"""

    def __init__(self, sampling_rate = 32000, noise_rate = 1.0, noise_length = 1.0, Es_Ni = 0, Pr = 0, F_of = 0):  # only default arguments here
        """
        Parameters:
        sampling rate (Hz): Needed to calculate length of noise frame and wait frame
        noise_rate (arrivals/second): noise packet rate /s (can overlap up to 3 packets)
        noise length (seconds): length of interference packet (if too large may just overlap constantly)
        Es_Ni(dBm): Desired Es/Ni (Es is hard coded internally)
        Pr(dB): estimate of received power from sending a digital normalized signal 
        F_of (Hz): offset of OSLA from center of BLE noise (-+). Used in in-band normalization calculation.
        analog_gain (dB): To prevent saturation, set analog gain accordingly
        """

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

        #######################
        #Calculate desired P_I#
        #######################

        #Bandwidth of OSLA signal hardcoded here
        #336 samples/chip
        #32 chips/symbol
        #200000000 samples/second
        #W = 1/R
        Ts = 336*32/200000000
        BW = 2/Ts
        
        #calculate desired interference power to be used in log normal distribution

        Es = -171
        P_I = Es-Es_Ni + 10*np.log10(BW)
        self.mu = P_I - 10*np.log10(noise_rate) - 10*np.log10(noise_length)

        self.Pr = Pr
        
        ########################
        #Normalize In-Band Gain#
        ########################

        #generate normalizer gain from LUT
        file_path = '/n/deer/z/samnolan/BLE_PSD.csv'
        try:
            PSD = np.genfromtxt(file_path,delimiter=',', dtype=np.double)
        except FileNotFoundError:
            print(f"Error: File not found at {file_path}")


        fbin_width = 100
        #define gain to have normalized in band power
        #With this gain, we will have normalized digital power and receive ~Pr that we measured
        PSD_i_low  = int(np.round((len(PSD)/2) + (F_of/fbin_width) - ((BW/fbin_width)/2)) - 20)
        PSD_i_high = int(np.round((len(PSD)/2) + (F_of/fbin_width) + ((BW/fbin_width)/2)) + 20)    

        self.G = np.sqrt(1/(fbin_width*np.sum(PSD[PSD_i_low:PSD_i_high,0])))
        #self.G = 0
        ##############################
        #set up interferer parameters#
        ##############################
        #phase shift
        self.theta = np.random.uniform(size=len(self.n_counters))*2j*np.pi

        self.sampling_rate = sampling_rate
        self.rate = noise_rate
        self.noise_length = noise_length

        self.noise_frame = np.round((sampling_rate*self.noise_length),0)

        self.arrival_clk = 0
        self.wait_frame = round(((-1/self.rate)*np.log(np.random.uniform())*self.sampling_rate))
        
        self.idx = 0
        self.starved = False
        ########################

        self.DebugPortName = 'Debug'
        self.message_port_register_out(pmt.intern(self.DebugPortName))


    def work(self, input_items, output_items):
        self.arrival_clk = self.arrival_clk + len(output_items[0])
        self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
        if (self.idx == None) and (self.arrival_clk >= self.wait_frame):
            self.starved = True
        '''
        #use this to send messages from the debug port
            PMT_msg = pmt.from_bool(self.state)
            self.message_port_pub(pmt.intern(self.DebugPortName), PMT_msg)
        '''
        #run arrival clk and signal when packet should start transmitting
        if ((self.arrival_clk >= self.wait_frame) and not self.starved):
            self.arrival_clk = self.arrival_clk - self.wait_frame
            self.wait_frame = round(((-1/self.rate)*np.log(np.random.uniform())*self.sampling_rate))
            #Mark Interferer to start
            self.n_counters[self.idx][0] = True
            #generate gain value from parameters
            P = np.random.normal(loc=self.mu, scale=0)
            self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-self.Pr)/10))
            #generate new phase offset
            self.theta[self.idx] = np.random.uniform()*2j*np.pi
            self.n_counters[self.idx][3] = self.arrival_clk



            #if next wait_frame occurs in same window
            self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
            if ((self.arrival_clk >= self.wait_frame) and (self.idx != None)):
                self.arrival_clk = self.arrival_clk - self.wait_frame
                self.wait_frame = round(((-1/self.rate)*np.log(np.random.uniform())*self.sampling_rate))
                #Mark Interferer to start
                self.n_counters[self.idx][0] = True
                #generate gain value from parameters
                P = np.random.normal(loc=self.mu, scale=0)
                self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-self.Pr)/10))
                #generate new phase offset
                self.theta[self.idx] = np.random.uniform()*2j*np.pi
                self.n_counters[self.idx][3] = self.arrival_clk

                #if all three interferers occur in the same window
                self.idx = next((i for i,j in enumerate(self.n_counters) if not j[0]),None)
                if ((self.arrival_clk >= self.wait_frame) and (self.idx != None)):
                    self.arrival_clk = self.arrival_clk - self.wait_frame
                    self.wait_frame = round(((-1/self.rate)*np.log(np.random.uniform())*self.sampling_rate))
                    #Mark Interferer to start
                    self.n_counters[self.idx][0] = True
                    #generate gain value from parameters
                    P = np.random.normal(loc=self.mu, scale=10)
                    self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-self.Pr)/10))
                    #generate new phase offset
                    self.theta[self.idx] = np.random.uniform()*2j*np.pi
                    self.n_counters[self.idx][3] = self.arrival_clk
            
            #PMT_msg = pmt.to_pmt(self.wait_frame)
            #self.message_port_pub(pmt.intern(self.DebugPortName), PMT_msg)

        #If we are starved we want to start noise immediately and also reset the arrival clock to prevent error buildup
        if ((self.idx != None) and self.starved):
            self.arrival_clk = len(output_items[0])
            self.wait_frame = round(((-1/self.rate)*np.log(np.random.uniform())*self.sampling_rate))
            #Mark Interferer to start
            self.n_counters[self.idx][0] = True
            #generate gain value from parameters
            P = np.random.normal(loc=self.mu, scale=0)
            self.n_counters[self.idx][2] = self.G*np.sqrt(10**((P-self.Pr)/10))
            #generate new phase offset
            self.theta[self.idx] = np.random.uniform()*2j*np.pi
            self.starved = False

            
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
        #    return -1
        return len(output_items[0][:])
