f = fopen('../../gnuradio/log/4_3_25/c16_wire_continuous-125dB_10M.bin', 'rb');
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
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d(1:1e6-1)/L))));
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%%
%Plot fft of all points
figure();
Fs = 10e6;
L = length(d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d/L))));
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")


%mix down these points

%%
t = (1/Fs)*(0:size(d)-1);
Fif = 200e6/336;

%downconvert
x_t = exp(-1j*2*pi*Fif*t);

d_dcon = x_t.'.*d;

%Create moving average filter
%%
%Create moving average filter
% mbw = 200e6/(32*336);
% mduration = 1/mbw;
% mfilter = ones(1,round(mduration*Fs));
% mfilter = mfilter/length(mfilter);
% 
% mfilter_padded = [mfilter zeros(1,length(d_dcon) - 1)];
% fmf = fft(mfilter_padded);
% 
% figure();
% Fs = 10e6;
% L = length(fmf);
% plot(Fs/L*(-L/2:L/2-1),(abs(fftshift(fmf/length(fmf)))));
% title("fft Spectrum of average filter")
% xlabel("f (Hz)")
% ylabel("|fft(X)|")
% 
% d_dcon_padded = [d_dcon.' zeros(1,length(mfilter) - 1)];
% fw = fft(d_dcon_padded);
% filtered_noise = ifft(fmf.*fw);
% P_av_filtered  = 10*log10(sum(abs(filtered_noise).^2)/length(filtered_noise))-137;

%%
%do pwelch to obtain average in-band power
Ts = 336*32/200000000;
BW = 2/Ts;

[sxx,f] = pwelch(d_dcon,500,0,Fs/100,Fs,'centered');
p_idx = (f >= -BW/2) & (f <= BW/2);
disp(10*log10(sum(sxx(p_idx))*100)-137);
