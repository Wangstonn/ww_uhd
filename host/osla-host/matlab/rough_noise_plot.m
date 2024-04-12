%%
variancesArray = [0.00832579, 0.00912519, 0.0124691, 0.00962529, 0.0153984, 0.00772279];
plot(variancesArray)
mean(variancesArray)

%%
snr_2 = 27;
snr_1 = 27+10*log10(4);
dists = 1:50;
snrs = snr_1-10*log10(dists.^2);
plot(dists, snrs)
title("Estimated SNR vs Distance with tx gain = 0 dB")
xlabel("Distance (m)")
ylabel("Estimated SNR (dB)")
