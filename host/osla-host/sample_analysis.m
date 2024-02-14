%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23
addpath('..\..\..\..\fp_emulator\')
addpath('..\..\..\..\\')
clearvars; close all; clc;

%% Plot the samples

% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("usrp_samples.prmbl.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d = (data(1,:)+j*data(2,:))*2^-8; %dest fixed point is 16,8
N_w = length(d); %length of window

figure(); grid on;
plot(real(d));
title("I Samples");

figure(); grid on;
plot(imag(d));
title("Q Samples");

% figure();
% plot(1/length(fft(d)) * abs(fft(d)).^2);

figure();
qqplot(real(d)); %see if its just noise
%% Processing 
% read preamble
fid = fopen("../../../matlab/mlsr/preamble.mem");
preamble_bits = fscanf(fid, '%1d')';
fclose(fid);

prmbl_amp = (1-2^-15);
prmbl_samps = repelem(2*(preamble_bits-.5)*prmbl_amp,32);
N_prmbl = length(prmbl_samps);

% cross correllation
[r,lags] = xcorr(d,prmbl_samps); %Turns out matlab implements xcorr in the with the order of arguments reversed
%second ardument of xcorr is conjugated and dragged
% xcorr([1],[1,1,1]) %to understand xcorr

figure(); grid on; hold on;
r_mag = abs(r);
plot(lags,r_mag/N_w);
title("xcorr mag");

[~,max_idx] = max(abs(r));
scatter(lags(max_idx),r_mag(max_idx)/N_w);
%t_hat is how much longer it takes pramble after start. For correct
%operation, should be -4 (src datapath takes 4 cycles)
D_hat = lags(max_idx);
if D_hat > 0
    N_samps_cap = (N_w - D_hat); %normalization of r with windowing
elseif D_hat < N_w - N_prmbl
    N_samps_cap = (N_prmbl+D_hat);
else
    N_samps_cap = N_w;
end

h_hat = r(max_idx)/(N_samps_cap * prmbl_amp^2);
h_hat_mag = abs(h_hat)
phi_hat = angle(h_hat)

D_comp = D_hat + 4; %dest start time is D_comp +src start
if D_comp < 0
    dest_delay = -D_comp
    src_delay = 0
else
    dest_delay = 0
    src_delay = D_comp
end

dest_ch_eq_re=shift_fp(cos(angle(h_hat))/abs(h_hat),0,14,11);
dest_ch_eq_im=shift_fp(sin(angle(h_hat))/abs(h_hat),0,14,11);

dest_ch_eq_re_bit=dec2bin_str(dest_ch_eq_re,14,11,1)
dest_ch_eq_im_bit=dec2bin_str(dest_ch_eq_im,14,11,1)

x_hat = d * 1/h_hat; %should match preamble

%% Noise estimation
fid = fopen("usrp_samples.noise.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d_noise = (data(1,:)+j*data(2,:))*2^-8; %dest fixed point is 16,8
n_var = var(d_noise)


esn0 = 10*log10((h_hat_mag)^2/(n_var*2))

%% Plot the samples
% addpath('..\..\..\..\fp_emulator\')
% addpath('..\..\..\..\\')
% clearvars; close all; clc;
% 
% % Read data
% % format depends on CPU Data Format Specification
% % https://files.ettus.com/manual/page_configuration.html 
% fid = fopen("usrp_samples.wired.noise.dat");
% data = fread(fid, [2, inf], 'int16');
% fclose(fid);
% %data is 2x#samples captured. I corresponds to the first row, Q second
% %The binary file format is simply a single line with I and Q alternating
% %e.g I1 Q1 I2 Q2 I3 Q3...
% 
% d = (data(1,:)+j*data(2,:))*2^-8; %dest fixed point is 16,8
% N_w = length(d); %length of window
% 
% figure(); grid on;
% plot(real(d));
% title("I Samples");
% 
% figure(); grid on;
% plot(imag(d));
% title("Q Samples");
% 
% % figure();
% % plot(1/length(fft(d)) * abs(fft(d)).^2);
% 
% figure();
% qqplot(real(d)); %see if its just noise
% 
% % point estimate of variance-assume noise in I and j have same variance
% var_hat = var(d);

% 
% %%
% plot(real(d))
% hold on
% %plot(prmbl_samps)
