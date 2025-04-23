%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23

clearvars; close all; fclose('all');
%% Noise estimation
ACCUM_WIDTH = 48;
ACCUM_FRAC = 19;
CAP_WIDTH = 32;
kChipFrac = ACCUM_FRAC-(ACCUM_WIDTH-32);

fid = fopen("./../data/fwd_p2p_noise_chips.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%32 bit data is split between two vectors. 
d_noise = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
N_w = length(d_noise); %length of window

figure();
grid on;
plot(d_noise);
title(['Chip Noise capture (Mean: ', num2str(mean(d_noise), '%.2f'), ', Variance: ', num2str(var(d_noise), '%.2f'), ')']);

figure()
d_noise = d_noise(1:end);
Fs = 200e6/336;
L = length(d_noise);
stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d_noise)))))
title("fft of noise")
xlabel("f (Hz)")
ylabel("|fft(X)| (db)")

var = var(d_noise)
% EsN0 = 10*log10((h_hat_mag * 336 * 32)^2/(2*var))

% figure();
% plot(1/length(fft(d)) * abs(fft(d)).^2);

figure();
qqplot(d_noise); %see if its just noise

figure();
histogram(d_noise)