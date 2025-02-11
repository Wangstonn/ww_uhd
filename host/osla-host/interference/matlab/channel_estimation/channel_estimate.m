%% Interference Sample acquisition visualization
% Author: Samuel Nolan
% email: samnolan@umich.edu
% 1/11/25

clearvars; close all; clc; fclose('all');
%%
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("dest_interf_samps.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
AdcFrac = 6;
d_raw = (data(1,:)+j*data(2,:));
d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
N_w = length(d); %length of window
fs = 200e6;
t1 = (0:length(d)-1)*1/200;

figure(); grid on;
plot(t1,real(d));
title("I Samples");
xlabel("time (us)");
ylabel("Amplitude");

figure(); grid on;
plot(imag(d));
title("Q Samples");

figure();
Fs = 200e6;
L = length(d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%I don't think I need to worry about amp here
%prmbl_amp = 1-2^-15;

%We're sending signal with max amplitude
%B = 2^15 - 1;

t = 0:N_w-1;

%downconvert
x_t = exp(-1j*2*pi/336*t);

downcon = (d_raw.*x_t);
%estim_amp = abs(sum((downcon))/(N_w));

A = sum(abs(d_raw))/(N_w);
P_r = 20*log10(estim_amp)-137;

%P_r = 20*log10(estim_amp)-137;

%compare to half power (should be 3 db less)
fidh = fopen("dest_interf_samps_half.dat");
datah = fread(fidh, [2, inf], 'int16');
fclose(fidh);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
dhalf_raw = (datah(1,:)+j*datah(2,:));



A_h = sum((dhalf_raw))/(N_w);

P_r_half = 20*log10(A_h)-137;

figure(); grid on;
plot(t1,real(dhalf_raw*2^-AdcFrac));
title("I Samples (half power)");
xlabel("time (us)");
ylabel("Amplitude");

figure(); grid on;
plot(real(downcon));





