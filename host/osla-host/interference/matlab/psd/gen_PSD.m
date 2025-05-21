f = fopen('gaussian_noise.bin', 'rb');
values = fread(f, Inf,'float32');
d = values(1:2:end) + values(2:2:end)*1j;

%%
%Pwelch table
Fs = 10e6;
resolution = 100;
fr = 1.3*1e6;
idx = fr/resolution;
[sxx, f] = pwelch(d,500,0,Fs/resolution,Fs,'centered');
pwelch(d,500,0,Fs/resolution,Fs,'centered');
hold on;

sxx_cut = sxx(((length(sxx)/2) - idx):((length(sxx)/2) + idx - 1));
f_cut = f(((length(f)/2) - idx):((length(f)/2) + idx - 1));

psd_data = [sxx_cut,f_cut];

writematrix(psd_data,'gaussian_PSD.csv');
%%
N = length(d);
bin_width = Fs / N;                       % Frequency bin width (Hz)

fft_vals = fftshift(fft(d));         % Centered FFT
P = abs(fft_vals / N).^2;                 % Power spectrum

f = linspace(-Fs/2, Fs/2, N); % Centered frequency axis

Ts = 336*32/200000000;

BW = 2/Ts;

idx_band = (f >= -BW/2) & (f <= BW/2);  % Index of bins in band

disp(sum(P(idx_band)) * bin_width); % Multiply by bin width

p_idx = (f_cut >= -BW/2) & (f_cut <= BW/2);
disp(sum(sxx_cut(p_idx))*100);





 




