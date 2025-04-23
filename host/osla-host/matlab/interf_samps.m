%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23

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

d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 14,6
N_w = length(d); %length of window

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
Fs = 200e6;
L = length(d);
figure();
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")