bits = randi([0, 1], 1000000, 1);
bit_rate = 1e6;
Fs = 10e6;
Fs_M = Fs/1e6;
modulation_idx = 0.5;
carrier_freq = 0;
wf = BLEmodulate(bits, bit_rate, Fs, modulation_idx, carrier_freq);

%%
%plot wf
t = (1/Fs)*(0:size(wf)-1);
figure(); 
plot(t(1:1e6)*1e3,real(wf(1:1e6)));
title('Experimental Interference Time-Domain');
xlabel('time (ms)');
ylabel("real(Amplitude)");

%Plot fft of 1e6 points
figure();
L = length(wf(1:1e6-1));
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(wf(1:1e6-1)/L))));
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")

%%
%add frequency offset
Fof = 0;
t = (1/Fs)*(0:size(wf)-1);

%shift
x_t = exp(1j*2*pi*Fof*t);

wf = x_t.'.*wf;

%Create moving average filter

mbw = 200e6/(32*336);
mduration = 1/mbw;
mfilter = ones(1,round(mduration*Fs));
mfilter = mfilter/length(mfilter);

mfilter_padded = [mfilter zeros(1,length(wf) - 1)];
fmf = fft(mfilter_padded);

wf_padded = [wf.' zeros(1,length(mfilter) - 1)];
fw = fft(wf_padded);
filtered_wf = ifft(fmf.*fw);

%Calculate Power estimate
power=sum(abs(filtered_wf).^2)/length(filtered_wf);
