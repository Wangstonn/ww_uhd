%% plot_samps - plot samples
%
% Description:
%   Loads samples and visualizes them
%
% Author: Winston Wang
% Date: 6-23

clearvars; close all; fclose('all');
AdcFrac = 6;

fwd_file = "../data/dest_interf_samps.dat";
% fwd_file = "../data/fb_p2p_prmbl_samps.dat";

%% Plot the samples
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen(fwd_file);
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 14,6
N_w = length(d); %length of window

OSR = 336;
t = 1:length(d);
if_tone = exp(1i*(2*pi/OSR*(t-1)));

figure(); grid on;
plot(real(d));
title("I Samples");
hold on;
% plot(real(if_tone));
% plot(-real(if_tone));

figure(); grid on;
plot(imag(d));
title("Q Samples");
hold on;
% plot(imag(if_tone));
% plot(imag(-if_tone));


figure();
Fs = 200e6;
L = length(d);
plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d))))
title("fft Spectrum of captured samples")
xlabel("f (Hz)")
ylabel("|fft(X)|")


% figure();
% qqplot(real(d)); %see if its just noise
% %% Processing 
% % Downconvert samples
% d_if = d.*conj(if_tone); 
% 
% figure(); grid on;
% plot(real(d_if));
% title("Downconverted I Samples");
% 
% figure(); grid on;
% plot(imag(d_if));
% title("Downconverted Q Samples");
% 
% % figure();
% % plot(Fs/L*(-L/2:L/2-1),abs(fftshift(fft(d_if))))
% % title("fft Spectrum of captured samples")
% % xlabel("f (Hz)")
% % ylabel("|fft(X)|")
% 
% 
% 
% % read preamble
% fid = fopen("preamble.mem");
% preamble_bits = fscanf(fid, '%1d')';
% fclose(fid);
% 
% prmbl_samps = repelem(2*(preamble_bits-.5),32);
% N_prmbl = length(prmbl_samps);
% 
% % EsN0 = 10^(4/10);
% % % Theoretical test-add noise
% % sigma_n = sqrt(N_prmbl/(2*EsN0));
% % 
% % d = prmbl_samps + sigma_n*randn(1,N_prmbl)+j*sigma_n*randn(1,N_prmbl);
% % d = prmbl_samps;
% 
% % cross correllation
% [r,lags] = xcorr(d_if,prmbl_samps); %Turns out matlab implements xcorr with the order of arguments reversed
% %second argument of xcorr is conjugated and dragged
% % xcorr([1],[1,1,1]) %to understand xcorr
% 
% figure(); grid on; hold on;
% r_mag = abs(r);
% stem(lags,r_mag);
% title("xcorr mag");
% 
% [~,max_idx] = max(abs(r));
% scatter(lags(max_idx),r_mag(max_idx));
% %t_hat is how much longer it takes pramble after start. For correct
% %operation, should be -4 (src datapath takes 4 cycles)
% D_hat = lags(max_idx);
% 
% N_w = length(d_if);
% 
% if N_w > N_prmbl
%     if D_hat < 0
%         N_samps_cap = max(0, N_prmbl + D_hat);
%     elseif D_hat < (N_w - N_prmbl)
%         N_samps_cap = N_prmbl;
%     else
%         N_samps_cap = max(0, N_prmbl - (D_hat - (N_w - N_prmbl)));
%     end
% else
%     if D_hat < 0
%         N_samps_cap = max(0, N_w + D_hat);
%     elseif D_hat < (N_prmbl - N_w)
%         N_samps_cap = N_w;
%     else
%         N_samps_cap = max(0, N_w - (D_hat - (N_prmbl - N_w)));
%     end
% end
% 
% % %The size of the peak depends on the number of preamble samples captured in
% % %the window.
% % if D_hat > 0
% %     N_samps_cap = (N_w - D_hat); %normalization of r with windowing
% % elseif D_hat < N_w - N_prmbl
% %     N_samps_cap = (N_prmbl+D_hat);
% % else
% %     N_samps_cap = N_w;
% % end
% 
% % Find the fading coefficient from the matched filter
% % The if_tone we generated is not synchronized to the tx if tone. After we
% % find the delay, this allows us to synchronize the two tones. The
% % remaining phase difference is caused by the channel.
% h_hat = exp(1i*(2*pi/OSR*(D_hat)))*r(max_idx)/(N_samps_cap);
% D_hat
% h_hat_mag = abs(h_hat)
% phi_hat = angle(h_hat)

% D_comp = D_hat + 4; %dest start time is D_comp +src start
% if D_comp < 0
%     dest_delay = -D_comp
%     src_delay = 0
% else
%     dest_delay = 0
%     src_delay = D_comp
% end

% dest_ch_eq_re=shift_fp(cos(angle(h_hat))/abs(h_hat),0,14,11);
% dest_ch_eq_im=shift_fp(sin(angle(h_hat))/abs(h_hat),0,14,11);
% 
% dest_ch_eq_re_bit=dec2bin_str(dest_ch_eq_re,14,11,1)
% dest_ch_eq_im_bit=dec2bin_str(dest_ch_eq_im,14,11,1)
% 
% x_hat = d_if * exp(1i*(2*pi/OSR*(D_hat))) * 1/h_hat; %should match preamble
% 
% % Align prmbl_samps with x_hat based on D_hat
% if D_hat >= 0
%     % Positive delay: shift prmbl_samps to the right by adding zeros at the beginning
%     aligned_prmbl_samps = [zeros(1, D_hat), prmbl_samps];
% else
%     % Negative delay: shift prmbl_samps to the left by adding zeros at the end
%     aligned_prmbl_samps = [prmbl_samps, zeros(1, abs(D_hat))];
%     x_hat = [zeros(1,abs(D_hat)),x_hat];
% end
% 
% 
% 
% figure()
% plot(real(x_hat),'DisplayName',"real(r/h)")
% hold on;
% plot(aligned_prmbl_samps,'DisplayName',"aligned preamble samples")
% title("Flatfading compensated signal vs preamble")
% legend
% 
% figure()
% plot(imag(x_hat),'DisplayName',"imag(r/h)")
% hold on;
% plot(aligned_prmbl_samps,'DisplayName',"aligned preamble samples")
% title("Flatfading compensated signal vs preamble")
% legend
% 
% %% Noise estimation
% ACCUM_WIDTH = 42;
% ACCUM_FRAC = 16;
% CAP_WIDTH = 32;
% kChipFrac = ACCUM_FRAC-(ACCUM_WIDTH-32);
% 
% fid = fopen("./../data/fwd_p2p_noise_chips.dat");
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
% 
% var = var(d_noise)
% EsN0 = 10*log10((h_hat_mag * 336 * 32)^2/(2*var))
% 
% %% Plot the samples
% % addpath('..\..\..\..\fp_emulator\')
% % addpath('..\..\..\..\\')
% % clearvars; close all; clc;
% % 
% % % Read data
% % % format depends on CPU Data Format Specification
% % % https://files.ettus.com/manual/page_configuration.html 
% % fid = fopen("usrp_samples.wired.noise.dat");
% % data = fread(fid, [2, inf], 'int16');
% % fclose(fid);
% % %data is 2x#samples captured. I corresponds to the first row, Q second
% % %The binary file format is simply a single line with I and Q alternating
% % %e.g I1 Q1 I2 Q2 I3 Q3...
% % 
% % d = (data(1,:)+j*data(2,:))*2^-8; %dest fixed point is 16,8
% % N_w = length(d); %length of window
% % 
% % figure(); grid on;
% % plot(real(d));
% % title("I Samples");
% % 
% % figure(); grid on;
% % plot(imag(d));
% % title("Q Samples");
% % 
% % % figure();
% % % plot(1/length(fft(d)) * abs(fft(d)).^2);
% % 
% % figure();
% % qqplot(real(d)); %see if its just noise
% % 
% % % point estimate of variance-assume noise in I and j have same variance
% % var_hat = var(d);
% 
% % 
% % %%
% % plot(real(d))
% % hold on
% % %plot(prmbl_samps)
% %plot(xcorr(prmbl_samps,prmbl_samps))
