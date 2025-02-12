
file = '..\..\gnuradio\log\c16_freq1k_32k.bin';

f = fopen(file, 'rb');
values = fread(f, Inf,'short');
complex_v = values(1:2:end) + values(2:2:end)*1j;

t = 0:size(complex_v,1)-1;

%downconvert
x_t = exp(-1j*2*pi/336*t);

downcon = (complex_v.*x_t);
%estim_amp = abs(sum((downcon))/(N_w));

A = abs(sum(complex_v))/size(complex_v,1);
P_r = 20*log10(estim_amp)-137;

