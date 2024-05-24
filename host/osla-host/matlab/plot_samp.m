%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23

clearvars; close all; clc; fclose('all');

%% MMIO Plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("fb_dlb_samps.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
AdcFrac = 12;
d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
N_w = length(d); %length of window

figure();
subplot(2,2,1)
sgtitle("Analog loopback capture")
grid on;
plot(real(d));
title("src rx I Samples");

subplot(2,2,2); grid on;
plot(imag(d));
title("src rx Q Samples");


fid = fopen("fwd_dlb_samps.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
AdcFrac = 6;
d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8

subplot(2,2,3); grid on;
plot(real(d));
title("dest rx I Samples");

subplot(2,2,4); grid on;
plot(imag(d));
title("dest rx Q Samples");
%% Stream Plot

% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("stream_samps.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
AdcFrac = 6; %dest
d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
N_w = length(d); %length of window

figure();
subplot(2,2,1)
sgtitle("Analog loopback capture")
grid on;
plot(real(d));
title("dest rx I Samples");

subplot(2,2,2); grid on;
plot(imag(d));
title("dest rx Q Samples");


% fid = fopen("fwd_alb_samps.dat");
% data = fread(fid, [2, inf], 'int16');
% fclose(fid);
% %data is 2x#samples captured. I corresponds to the first row, Q second
% %The binary file format is simply a single line with I and Q alternating
% %e.g I1 Q1 I2 Q2 I3 Q3...
% AdcFrac = 6;
% d = (data(1,:)+j*data(2,:))*2^-AdcFrac; %dest fixed point is 16,8
% 
% subplot(2,2,3); grid on;
% plot(real(d));
% title("dest rx I Samples");
% 
% subplot(2,2,4); grid on;
% plot(imag(d));
% title("dest rx Q Samples");


%% Prmbl Plot
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("../data/dlb_prmbl_stream_01.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...
AdcFrac = 0;
d = (data(1,:)+j*data(2,:))*2^-AdcFrac;
N_w = length(d); %length of window

figure();
subplot(2,1,1)
sgtitle("Digital loopback preamble stream")
grid on;
plot(real(d));
title("dest rx I Samples");

subplot(2,1,2); grid on;
plot(imag(d));
title("dest rx Q Samples");

% fid = fopen("../data/dlb_prmbl_cap.dat");
% data = fread(fid, [2, inf], 'int16');
% fclose(fid);
% %data is 2x#samples captured. I corresponds to the first row, Q second
% %The binary file format is simply a single line with I and Q alternating
% %e.g I1 Q1 I2 Q2 I3 Q3...
% AdcFrac = 6;
% d = (data(1,:)+j*data(2,:))*2^-AdcFrac; 
% N_w = length(d); %length of window
% 
% figure();
% subplot(2,1,1)
% sgtitle("Digital loopback preamble cap")
% grid on;
% plot(real(d));
% title("dest rx I Samples");
% 
% subplot(2,1,2); grid on;
% plot(imag(d));
% title("dest rx Q Samples");




