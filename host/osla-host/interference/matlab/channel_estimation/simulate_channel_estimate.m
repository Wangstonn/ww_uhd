clearvars; close all; clc; fclose('all');
%%
% Simulation
N_prmbl = 2^16 - 1;

%Generate correlation signal
f_if = 336;
t = 0:(N_prmbl-1);
x_t = exp(-1j*2*pi/f_if*t);

%Generate noisy data
theta = randn();
h = .5;
B = 2^16 - 1;

EsN0 = 10^(4/10);
% % Theoretical test-add noise
sigma_n = sqrt(N_prmbl/(2*EsN0));
%sigma_n = 0;

s_t = h*B*exp(1j*2*pi/f_if*t + 1j*2*pi*theta) + sigma_n*randn(1,N_prmbl)+1j*sigma_n*randn(1,N_prmbl);
%multiply and sum

NT = N_prmbl;
h_hat = sum(abs(s_t.*x_t))/(NT*B);

fprintf('h_hat = %d \n',h_hat);
fprintf('h = %d \n',h);


