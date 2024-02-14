%%
variancesArray = [0.00832579, 0.00912519, 0.0124691, 0.00962529, 0.0153984, 0.00772279];
plot(variancesArray)
mean(variancesArray)

%%
snr_1 = 13;
dists = 1:50;
snrs = 13-log(dists.^2);
plot(dists, snrs)
title("Estimated SNR vs Distance with tx gain = 0 dB")
xlabel("Distance (m)")
ylabel("Estimated SNR (dB)")
