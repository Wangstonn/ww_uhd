clear; clc;

f = fopen('../../gnuradio/log/c16_SingleTone_20M.bin', 'rb');
values = fread(f, Inf,'short');
d = values(1:2:end) + values(2:2:end)*1j;
%%
figure();
Fs = 10e6;
L = length(d(1:10e6-1));
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d(1:10e6-1)))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")
%%
t = (1/Fs)*(0:size(d)-1);

Fif = 200e6/336;

%downconvert
x_t = exp(-1j*2*pi*Fif*t);
figure();
Fs = 10e6;
L = length(d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(x_t))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")
%%
downcon = (x_t.'.*d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(downcon))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")
%estim_amp = abs(sum((downcon))/(N_w));

A = (abs(sum(downcon))/size(d,1));
P_r = 20*log10(A)-137;

