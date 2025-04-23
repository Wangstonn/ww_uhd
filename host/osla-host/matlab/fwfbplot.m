%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23

clearvars; close all; fclose('all');
ACCUM_WIDTH = 48;
ACCUM_FRAC = 19;
CAP_WIDTH = 32;
kChipFrac = ACCUM_FRAC-(ACCUM_WIDTH-32);
AdcFrac = 12;

fwd_file = "../data/fwd_p2p_chips1.dat";
fb_file = "../data/fb_p2p_samps1.dat";
%% fb plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen(fb_file);
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
N_w = length(d); %length of window

figure();
subplot(2,2,1)
sgtitle("Analog loopback capture")
grid on;
plot(real(d));
title("src rx I Samples");

subplot(2,2,2); grid on;
plot(imag(d));
title("src rx Q Samples");

% abs(sum(d(17953:17953+15))).^2


%% Fwd Plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen(fwd_file);
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%32 bit data is split between two vectors. 
d = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
N_w = length(d); %length of window

figure()
d = d(1:end);
Fs = 200e6/336;
L = length(d);
stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d)))))
title("fft Spectrum of captured chips")
xlabel("f (Hz)")
ylabel("|fft(X)| (dB)")

figure();
grid on;
plot(real(d));
title("dest rx I Chips");

% dfilt = conv(ones(1,32),d);
% dfilts = dfilt(32:32:length(dfilt));
% figure()
% plot(dfilt);
% hold on

% subplot(2,2,4); grid on;
% plot(imag(d));
% title("dest rx Q Chips");




%% Fwd Plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("../data/fwd_p2p_noise_chips.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%32 bit data is split between two vectors. 
d = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
N_w = length(d); %length of window


figure();
grid on;
plot(real(d));
title("dest rx I Noise Chips");



% subplot(2,2,4); grid on;
% plot(imag(d));
% title("dest rx Q Chips");

