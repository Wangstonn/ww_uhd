%% tone_estim - calibrate interferer power
%
% Description:
%   Loads sinusoid calibration data and estimates the power. This
%   calibrates the interferer to verify power measurments using ___ code
%   This uses USRP FPGA samples
%
% Author: Winston Wang
% Date: 6-23


clearvars; close all; fclose('all');
AdcFrac = 2;

fwd_file = "../data/interf_cal_samps.dat";

% fwd_file = "../data/dest_interf_samps.dat";
% fwd_file = "../data/fb_p2p_prmbl_samps.dat";

%% Plot the samples
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen(fwd_file);
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d = (data(1,:)+j*data(2,:))*2^-AdcFrac;
N_w = length(d); %length of window
fs = 200e6;

OSR = 336;
t = 1:length(d);
if_tone = exp(1i*(2*pi/OSR*(t-1)));

figure(); grid on;
plot(real(d));
title("I Samples");
hold on;
plot(real(if_tone));
plot(-real(if_tone));

figure(); grid on;
plot(imag(d));
title("Q Samples");
hold on;
plot(imag(if_tone));
plot(imag(-if_tone));


figure();
L = length(d);
plot(fs/L*(-L/2:L/2-1),abs(fftshift(1/fs*fft(d))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%% Analysis
d_if = d.*conj(if_tone);

figure();
fs = 200e6;
L = length(d);
plot(fs/L*(-L/2:L/2-1),abs(fftshift(1/fs*fft(d_if))))
title("fft Spectrum of downconverted samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

h_hat_mag = mean(abs(d_if))

plot_idx = 1:length(d_if); %seconds
t = (1:L)/fs;
figure;
plot(t(plot_idx),real(d_if(plot_idx)),'DisplayName',"Downconverted I"); hold on;
plot(t(plot_idx),imag(d_if(plot_idx)),'DisplayName',"Downconverted Q"); 

plot(t(plot_idx),abs(d_if(plot_idx)),'DisplayName',"Envelope");
title('Calibration Sinusoid, tx gain = 31.5dB');
legend;

%%
adc_to_rss_factor = -137 + 30; %dbm

h_hat_mag = mean(abs(d_if));
P_r = 20*log10(h_hat_mag)+adc_to_rss_factor

