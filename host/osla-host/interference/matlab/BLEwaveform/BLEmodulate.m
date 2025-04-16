function [signal] = BLEmodulate(bits, bit_rate, sample_rate, modulation_idx, carrier_freq)
%BLEMODULATE generate waveform for bluetooth LE
%   bits: array of bits, bit_rate: bit/second, sample_rate: sample/second,
%   modulation_idx: 0.5 for bluetooth LE, carrier_freq: Hz
sample_per_bit = ceil(sample_rate / bit_rate);
symbols = - bits * 2 + 1;
upsampled_symbols = repelem(symbols, sample_per_bit);
instant_freq = upsampled_symbols * (modulation_idx * bit_rate / sample_rate * pi);
lpf = Gaussian_Filter(modulation_idx, 6, sample_per_bit);
lpf_freq = conv(instant_freq, lpf, 'same');
shifted_freq = lpf_freq + carrier_freq / sample_rate;
signal = exp(1j * cumsum(shifted_freq));
end

