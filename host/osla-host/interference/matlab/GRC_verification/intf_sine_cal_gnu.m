%% intf_sine_cal_gnu - calibrate interferer power
%
% Description:
%   Loads sinusoid calibration data and estimates the power. This
%   calibrates the interferer to verify power measurments using ___ code
%
% Author: Winston Wang
% Date: 6-23

clear all; close all;

fid = fopen('../../gnuradio/grc_graphs/data/c16_interf_10M.bin', 'rb');
data = fread(fid, Inf,'short');
fclose(fid);
d = data(2:2:end) + data(1:2:end)*1j;
d = d';
fs = 10e6;

%%
L = length(d);
figure;
plot(fs/L*(-L/2:L/2-1),10*log10(abs(fftshift(1/fs*fft(d)))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

plot_idx = 1:floor(0.001*fs); %seconds
t = (1:L)/fs;

figure(); grid on;
plot(t(plot_idx),real(d(plot_idx)));
title("I Samples");
hold on;
% plot(real(if_tone));
% plot(-real(if_tone));

figure(); grid on;
plot(t(plot_idx),imag(d(plot_idx)));
title("Q Samples");
hold on;
% plot(imag(if_tone));
% plot(imag(-if_tone));

%% Downconvert and visualize
L = length(d);
OSR = 336;
f_if = 200e6/336;
t = (1:L)/fs;

if_tone = exp(1j*2*pi*f_if*t);
d_if = d.*conj(if_tone);

figure;
plot(fs/L*(-L/2:L/2-1),10*log10(abs(fftshift(1/fs*fft(d_if)))))
title("fft Spectrum of downconverted samples")
xlabel("f (Hz)")
ylabel("|fft(X)| (db)")
%estim_amp = abs(sum((downcon))/(N_w));

figure;
plot(t(plot_idx),real(d_if(plot_idx)),'DisplayName',"Downconverted I"); hold on;
plot(t(plot_idx),imag(d_if(plot_idx)),'DisplayName',"Downconverted Q"); 

plot(t(plot_idx),abs(d_if(plot_idx)),'DisplayName',"Envelope");
title('Calibration Sinusoid, tx gain = 31.5dB');
legend;

%% PSD
this.kIntfPktLen = 2e-3;
this.kClkPeriod = 5e-9;%-9
this.kArrivalMu = 5e-3;
this.kLognormVar = 0; %db,10
this.OSR = 336;
this.T = 32;
kIfFreq = 1/(this.OSR*this.kClkPeriod);

fc = kIfFreq;
BW = 2/(this.T*this.OSR*this.kClkPeriod);
% resolution = 100;
% % fr = 1.3*1e6;
% % idx = fr/resolution;
% [sxx, f] = pwelch(noise_vec,500,0,Fs/resolution,Fs,'centered');
nfft = 2^17;              % Large enough for high frequency resolution
df = fs/nfft;
% binsInBw = bw/df

window = hamming(nfft);   % Use full-length window (optional: try shorter)
noverlap = nfft/2;        % 50% overlap

[Pxx, F] = pwelch(d, window, noverlap, nfft, fs);
% Shift frequency and PSD so that 0 Hz is centered
Pxx_shifted = fftshift(Pxx);
F_shifted = linspace(-fs/2, fs/2, length(Pxx));

% Plot PSD in dB
figure;
plot(F_shifted/1e3, 10*log10(Pxx_shifted), 'LineWidth', 1.2);
xlabel('Frequency (kHz)');
ylabel('PSD (dB/Hz)');
title('Welch Power Spectral Density Estimate');
grid on;
xlim([fc - BW, fc + BW]/1e3);  % Zoom around target band

% Highlight target band
hold on;
yl = ylim;
fill([fc-BW/2 fc+BW/2 fc+BW/2 fc-BW/2]/1e3, ...
     [yl(1) yl(1) yl(2) yl(2)], ...
     [0.9 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

legend('PSD estimate', 'Target band');

% Find low/high bin indices for integration
midpoint = floor(length(Pxx_shifted)/2);
PSD_i_low  = round(midpoint + (kIfFreq)/df - (BW/df)/2);
PSD_i_high = round(midpoint + (kIfFreq)/df + (BW/df)/2);

%%
adc_to_rss_factor = -137-20*log10(4)+30; %dbm

h_hat_mag = mean(abs(d_if));
P_r = 20*log10(h_hat_mag)+adc_to_rss_factor

