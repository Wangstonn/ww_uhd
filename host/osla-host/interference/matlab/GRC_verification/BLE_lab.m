clear; clc;

f = fopen('../../gnuradio/log/2_17_2025/c16_SingleTone_10M.bin', 'rb');
values = fread(f, Inf,'short');
d = values(1:2:end) + values(2:2:end)*1j;
%%
%plot time domain 100ms
%mix down these points
Fs = 10e6;
t = (1/Fs)*(0:size(d)-1);
figure(); 
plot(t(1:1e6)*1e3,real(d(1:1e6)));
title('Experimental Interference Time-Domain');
xlabel('time (ms)');
ylabel("real(Amplitude)");

%Plot fft of 1e6 points
figure();
L = length(d(1:1e6-1));
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d(1:1e6-1)))));
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%%
%Plot fft of all points
figure();
Fs = 10e6;
L = length(d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d))));
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%mix down these points
t = (1/Fs)*(0:size(d)-1);
%%
Fif = 200e6/336;

%downconvert
x_t = exp(-1j*2*pi*Fif*t);

d_dcon = x_t.'.*d;

%Create moving average filter

mbw = 200e6/(32*336);
mduration = 1/mbw;
mfilter = ones(1,round(mduration*Fs));
mfilter = mfilter/length(mfilter);

mfilter_padded = [mfilter zeros(1,length(d_dcon) - 1)];
fmf = fft(mfilter_padded);

d_dcon_padded = [d_dcon.' zeros(1,length(mfilter) - 1)];
fw = fft(d_dcon_padded);
filtered_noise = ifft(fmf.*fw);

%calculate power
P_av = 10*log10(sum(abs(filtered_noise).^2)/length(filtered_noise))-137;
disp(P_av);
