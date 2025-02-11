bits = randi([0, 1], 2000, 1);
bit_rate = 1e6;
Fs = 16e6;
Fs_M = Fs/1e6;
modulation_idx = 0.5;
carrier_freq = 0;
waveform = BLEmodulate(bits, bit_rate, Fs, modulation_idx, carrier_freq);

%%
wf = fftshift(fft(waveform));
L = length(waveform);
f_bins = Fs_M*(-L/2 : L/2 - 1)/L;
figure()
plot(f_bins,20*log10(abs(wf/L)));
xlabel('Frequency (MHz)');
ylabel('Relative Amplitude (dB)');
title('Bluetooth Frequency Spectrum');

sum(abs(wf/L).^2)

%%
resolution = 100;

padnum = Fs/resolution;
g = padnum/length(waveform);

waveform_padded = [waveform' zeros(1,padnum - length(waveform))];

wfp = g*fftshift(fft(waveform_padded));
Lp = length(wfp);
fp_bins = Fs_M*(-Lp/2 : Lp/2 - 1)/Lp;
figure();
plot(fp_bins,20*log10(abs(wfp/Lp)));
xlabel('Frequency (MHz)');
ylabel('Relative Amplitude (dB)');
title('Bluetooth Frequency Spectrum');

range = 1.3*1e6;
range_idx = range/resolution;


wf_cut = wfp((-range_idx + (length(wfp)/2)) : (range_idx + (length(wfp)/2) - 1));
figure();
plot(20*log10(abs(wf_cut/Lp)));
sum(abs(wf/L).^2)

%%
%Pwelch table
resolution = 100;
fr = 1.3*1e6;
idx = fr/resolution;
[sxx, f] = pwelch(waveform,1000,0,Fs/resolution,Fs,'centered');
pwelch(waveform,1000,0,Fs/resolution,Fs,'centered');

sxx_cut = sxx((length(sxx)/2) - idx:(length(sxx)/2) + idx - 1);
f_cut = f((length(f)/2) - idx:(length(f)/2) + idx - 1);

psd_data = [sxx_cut,f_cut];

writematrix(psd_data,'BLE_PSD.csv');


 




