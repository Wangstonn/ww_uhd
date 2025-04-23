%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23

clearvars; close all; fclose('all');
ACCUM_WIDTH = 48;
ACCUM_FRAC = 19;
CAP_WIDTH = 32;
kChipFrac = ACCUM_FRAC-(ACCUM_WIDTH-32);

%% Noise capture
% 
% fid = fopen("./../data/fwd_p2p_noise_samps.dat");
% data = fread(fid, [2, inf], 'int16');
% fclose(fid);
% 
% %32 bit data is split between two vectors. 
% d_noise = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
% N_w = length(d_noise); %length of window
% 
% figure();
% grid on;
% plot(d_noise);
% title(['Chip Noise capture (Mean: ', num2str(mean(d_noise), '%.2f'), ', Variance: ', num2str(var(d_noise), '%.2f'), ')']);
% 
% figure()
% d_noise = d_noise(1:end);
% Fs = 200e6/336;
% L = length(d_noise);
% stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d_noise)))))
% title("fft of noise")
% xlabel("f (Hz)")
% ylabel("|fft(X)| (db)")

%% SNR verification
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("./../data/fwd_p2p_chips5.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%32 bit data is split between two vectors. 
d = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
N_w = length(d); %length of window

figure();
grid on;
plot(d);
title(['Dest rx I chip capture (Mean: ', num2str(mean(d), '%.2f'), ', Variance: ', num2str(var(d), '%.2f'), ')']);

EsN0 = 10*log10(32*(mean(d))^2/(2*var(d)))

figure()
d = d(1:end);
Fs = 200e6/336;
L = length(d);
stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d)))./Fs))
title("fft Spectrum of captured sinusoid")
xlabel("f (Hz)")
ylabel("|fft(X)| (dB)")

dfilt = conv(ones(1,32),d);
dfilts = dfilt(32:32:length(dfilt));
figure()
plot(dfilt);
hold on
scatter(32:32:length(dfilt),dfilts);

figure()
qqplot(dfilt)

figure()
dfilts = dfilt(1:end);
Fs = 200e6/336;
L = length(dfilts);
stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(dfilts)))./Fs))
title("fft Spectrum of captured sinusoid")
xlabel("f (Hz)")
ylabel("|fft(X)| (dB)")



%% Active plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("./../data/fwd_p2p_chips5.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%32 bit data is split between two vectors. 
d = ((data(1,:))*2^16 + double(typecast(int16(data(2,:)),"uint16")))*2^-kChipFrac; 
N_w = length(d); %length of window

Ts = 5e-9 * 336;
t = (0:(length(d)-1)) * Ts * 1e3;
figure();
grid on;
plot(t,d);
title(['Dest rx I chip capture']);
ylabel("matched filter output")
xlabel("ms")


% figure()
% d = d(1:end);
% Fs = 200e6/336;
% L = length(d);
% stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d)))))
% title("fft Spectrum of captured sinusoid")
% xlabel("f (Hz)")
% ylabel("|fft(X)| (dB)")

%%
% 
% figure()
% d = d(1:end);
% Fs = 200e6/336;
% L = length(d);
% stem(Fs/L*(-L/2:L/2-1),20*log10(abs(fftshift(fft(d)))))
% title("fft Spectrum of captured sinusoid")
% xlabel("f (Hz)")
% ylabel("|fft(X)| (dB)")

% %%
% fid = fopen("fb_dlb_samps.dat");
% data = fread(fid, [2, inf], 'int16');
% fclose(fid);
% %data is 2x#samples captured. I corresponds to the first row, Q second
% %The binary file format is simply a single line with I and Q alternating
% %e.g I1 Q1 I2 Q2 I3 Q3...
% AdcFrac = 12;
% d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
% N_w = length(d); %length of window
% 
% figure();
% subplot(2,2,1)
% sgtitle("Analog loopback capture")
% grid on;
% plot(real(d));
% title("src rx I Samples");
% 
% subplot(2,2,2); grid on;
% plot(imag(d));
% title("src rx Q Samples");
% 
% 
% 
% %% Test
% 
% a = 305419896; %12345678
% b = 4660;
% c = 22136;
% 
% a_hat = b*2^16 + double(typecast(int16(c),"uint16"))
% 
% predicted_esn0 = [43.0924, 43.1036, 42.9037, 43.2465, 37.724, 37.5915, 32.762, 27.9838,28.096,...
%     28.3533, 29.6605, 30.6618, 31.9903, 35.1528, 47.9755, 44.8687, 40.7702, 23.1134,...
%     17.7571, 13.5052];
% 
% measured_esn0 = [42.6578,42.3406, 41.7727, 41.8542, 40.6238, 40.2187, 37.7862,34.2448,33.6568,...
%     34.7234, 35.6220, 36.5059, 37.3899, 39.5329, 43.3844, 42.4054, 42.3946, 32.7830,...
%     29.1303, 26.9563];
% % Create scatter plot
% figure();
% scatter(predicted_esn0, measured_esn0, 'filled');
% hold on;
% 
% % Add straight line through origin
% x = linspace(min(predicted_esn0), max(predicted_esn0), 100); % Generate x values for the line
% y = x; % Line with slope 1 and intercept 0
% plot(x, y, 'r--', 'LineWidth', 2); % Plot the line in red with dashed style
% 
% % Add labels and title
% xlabel('Predicted E_s/N_0');
% ylabel('Measured E_s/N_0');
% title('Predicted vs Measured E_s/N_0');
% legend('Data Points', 'Ideal Line', 'Location', 'best');
% grid on;
% hold off;

%% RX_gain
% rx_gain = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30];
% var = [29.4913, 29.385, 29.5782, 28.9859, 29.8356, 30.0203, 29.3961, 30.4743, 30.554, 30.9916, 31.4724, 32.3447, 32.9011, 34.0835, 35.9594, 36.9853, 39.5754, 41.4779, 45.2277, 48.711, 54.9174, 60.9974, 69.2489, 78.4937, 92.0707, 106.649, 127.236, 149.659, 182.996, 221.991, 269.37];
% figure();
% scatter(rx_gain, 20*log10(var), 'filled');
% hold on;
% xlabel('rx gain (db)');
% ylabel('noise variance (db)');

