%% Sample acquisition visualization
% Author: Winston Wang
% email: wangston@umich.edu
% 12/13/23
%addpath('..\..\..\..\fp_emulator\')
addpath('..\..\..\..\\')
clearvars; close all; clc;

%% Load samples
% Read data
% format depends on CPU Data Format Specification
% https://files.ettus.com/manual/page_configuration.html 
fid = fopen("usrp_samples.prmbl.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);
%data is 2x#samples captured. I corresponds to the first row, Q second
%The binary file format is simply a single line with I and Q alternating
%e.g I1 Q1 I2 Q2 I3 Q3...

d = (data(1,:)+j*data(2,:));
N_w = length(d); %length of window

figure(); grid on;
plot(real(d));
title("I Samples");

figure(); grid on;
plot(imag(d));
title("Q Samples");

%% Noise estimation
fid = fopen("usrp_samples.noise.dat");
data = fread(fid, [2, inf], 'int16');
fclose(fid);

%% read preamble
fid = fopen("../../../matlab/mlsr/preamble.mem");
preamble_bits = fscanf(fid, '%1d')';
fclose(fid);

prmbl_amp = 1; 
prmbl_samps = repelem(2*(preamble_bits-.5)*prmbl_amp,32);
N_prmbl = length(prmbl_samps);