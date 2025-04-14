bits = randi([0, 1], 1000000, 1);
bit_rate = 1e6;
Fs = 10e6;
Fs_M = Fs/1e6;
modulation_idx = 0.5;
carrier_freq = 0;
waveform = BLEmodulate(bits, bit_rate, Fs, modulation_idx, carrier_freq);

%%
%Pwelch table
resolution = 100;
fr = 1.3*1e6;
idx = fr/resolution;
[sxx, f] = pwelch(waveform,500,0,Fs/resolution,Fs,'centered');
pwelch(waveform,500,0,Fs/resolution,Fs,'centered');
hold on;

sxx_cut = sxx(((length(sxx)/2) - idx):((length(sxx)/2) + idx - 1));
f_cut = f(((length(f)/2) - idx):((length(f)/2) + idx - 1));

psd_data = [sxx_cut,f_cut];

writematrix(psd_data,'BLE_PSD.csv');
%%
N = length(waveform);
bin_width = Fs / N;                       % Frequency bin width (Hz)

fft_vals = fftshift(fft(waveform));         % Centered FFT
P = abs(fft_vals / N).^2;                 % Power spectrum

f = linspace(-Fs/2, Fs/2, N); % Centered frequency axis

Ts = 336*32/200000000;

BW = 2/Ts;

idx_band = (f >= -BW/2) & (f <= BW/2);  % Index of bins in band

disp(sum(P(idx_band)) * bin_width); % Multiply by bin width

p_idx = (f_cut >= -BW/2) & (f_cut <= BW/2);
disp(sum(sxx_cut(p_idx))*100);

wf = waveform;
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
disp(sum(abs(filtered_wf).^2)/length(filtered_wf));





 




