bits = randi([0, 1], 1000, 1);
bit_rate = 1e6;
Fs = 16e6;
Fs_M = Fs/1e6;
modulation_idx = 0.5;
carrier_freq = 0;
waveform = BLEmodulate(bits, bit_rate, Fs, modulation_idx, carrier_freq);
wf = fftshift(fft(waveform));
L = length(waveform);
f_bins = Fs_M*(-L/2 : L/2 - 1)/L;
figure()
plot(f_bins,20*log10(abs(wf/L)));
xlabel('Frequency (MHz)');
ylabel('Relative Amplitude (dB)');
title('Bluetooth Frequency Spectrum');

%pwelch(waveform,100,0,sample_rate/100,sample_rate,'centered');




mbw = 20*1000; %Khz
mduration = 1/mbw;
mfilter = ones(1,round(mduration*Fs)); 
mfilter_padded = [mfilter zeros(1,length(waveform) - 1)];
fmf = fft(mfilter_padded);
%figure();
%plot(fbins,10*log(abs(fmf/L)));
waveform_padded = [waveform' zeros(1,length(mfilter) - 1)];

theta = rand;

waveform_padded = waveform_padded*exp(j*2*pi*theta);

fw = fft(waveform_padded);

filtered_waveform = [ifft(fmf.*fw) zeros(1,5*length(waveform_padded))];
ffw = fftshift(fft(filtered_waveform));
L_f = length(ffw);
fbins_f = Fs_M*(10^3)*(-L_f/2:L_f/2-1)/L_f; %KHz

figure()
plot(fbins_f,20*log10(abs(ffw)/L_f));
xlabel('Frequency (kHz)');
ylabel('Relative Amplitude');
title('Bluetooth Frequency Spectrum match filtered');
xlim([-10,10]);

figure()
pwelch(ifft(fmf.*fw),100,0,Fs/100,Fs,'centered');
xlim([-.01,.01]);

%need shifted psd and fft

f1 = 1e6/4;
f2 = 2*f1;
f3 = 3*f1;
f4 = 4*f1;

t = (0:length(waveform_padded)-1)/(Fs);

f1t = exp(j*2*pi*f1*t);
f2t = exp(j*2*pi*f2*t);
f3t = exp(j*2*pi*f3*t);
f4t = exp(j*2*pi*f4*t);

wvf1 = waveform_padded.*f1t;
wvf2 = waveform_padded.*f2t;
wvf3 = waveform_padded.*f3t;
wvf4 = waveform_padded.*f4t;

fw1 = fft(wvf1);
fw2 = fft(wvf2);
fw3 = fft(wvf3);
fw4 = fft(wvf4);

mfw1 = ifft(fmf.*fw1);
mfw2 = ifft(fmf.*fw2);
mfw3 = ifft(fmf.*fw3);
mfw4 = ifft(fmf.*fw4);

mfwp1 = [mfw1 zeros(1,5*length(waveform_padded))];
ffw1 = fftshift(fft(mfwp1));

mfwp2 = [mfw2 zeros(1,5*length(waveform_padded))];
ffw2 = fftshift(fft(mfwp2));

mfwp3 = [mfw3 zeros(1,5*length(waveform_padded))];
ffw3 = fftshift(fft(mfwp3));

mfwp4 = [mfw4 zeros(1,5*length(waveform_padded))];
ffw4 = fftshift(fft(mfwp4));

figure();
subplot(2, 2, 1); % 2x2 grid, first position
plot(fbins_f,20*log10(abs(ffw1)/L_f));
xlabel('Frequency (kHz)');
ylabel('Relative Amplitude (dB)');
title(['Frequency Spectrum Match Filtered f_o = ',num2str(f1/1e3),' kHz']);
xlim([-10,10]);

subplot(2, 2, 2); % 2x2 grid, first position
plot(fbins_f,20*log10(abs(ffw2)/L_f));
xlabel('Frequency (kHz)');
ylabel('Relative Amplitude (dB)');
title(['Frequency Spectrum Match Filtered f_o = ',num2str(f2/1e3),' kHz']);
xlim([-10,10]);

subplot(2, 2, 3); % 2x2 grid, first position
plot(fbins_f,20*log10(abs(ffw3)/L_f));
xlabel('Frequency (kHz)');
ylabel('Relative Amplitude (dB)');
title(['Frequency Spectrum Match Filtered f_o = ',num2str(f3/1e3),' kHz']);
xlim([-10,10]);

subplot(2, 2, 4); % 2x2 grid, first position
plot(fbins_f,20*log10(abs(ffw4)/L_f));
xlabel('Frequency (kHz)');
ylabel('Relative Amplitude (dB)');
title(['Frequency Spectrum Match Filtered f_o = ',num2str(f4/1e3),' kHz']);
xlim([-10,10]);


%need Amplitude histogram of matched filter output for frequency shifted
%version

%% Time domain Histogram
figure();
subplot(2, 2, 1); % 2x2 grid, first position
histogram(gca,real(mfw1),100);
%set(gca,'view',[90 -90]);
xlabel('Amplitude');
title(['Amplitude distribution f_o = ',num2str(f1/1e3),' kHz']);

subplot(2, 2, 2); % 2x2 grid, first position
histogram(gca,real(mfw2),100);
%set(gca,'view',[90 -90]);
xlabel('Amplitude');
title(['Amplitude distribution f_o = ',num2str(f2/1e3),' kHz']);

subplot(2, 2, 3); % 2x2 grid, first position
histogram(gca,real(mfw3),100);
%set(gca,'view',[90 -90]);
xlabel('Amplitude');
title(['Amplitude distribution f_o = ',num2str(f3/1e3),' kHz']);

subplot(2, 2, 4); % 2x2 grid, first position
histogram(gca,real(mfw4),100);
%set(gca,'view',[90 -90]);
xlabel('Amplitude');
title(['Amplitude distribution f_o = ',num2str(f4/1e3),' kHz']);
%%
figure()

histogram(gca,imag(ifft(fmf.*fw)),100);
hold on 
histogram(gca,real(ifft(fmf.*fw)),100);
legend('Q','I');
%set(gca,'view',[90 -90]);
xlabel('Amplitude');
title('Amplitude distribution f_o = 0 kHz');

