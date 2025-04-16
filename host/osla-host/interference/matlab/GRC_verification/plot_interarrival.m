%%
data = readmatrix("data/waveform.csv");
d = (data(:,1)+j*data(:,2));
A = readmatrix("data/interarrival_times.csv");
A = unique(A,'stable');

arrivals = zeros(size(A)-1);
arrivals(1) = A(1);

for i = 2:length(A)-1
    arrivals(i) = arrivals(i - 1) + A(i);
end

%arrivals_raw = zeros(size(A)-1);
%arrivals_raw(1) = A(1);

%for i = 2:length(A)-1
%    arrivals_raw(i) = arrivals_raw(i - 1) + A(i);
%end


figure(); grid on;
plot(real(d));
hold on;
title("Interference Packets")
plot(arrivals,zeros(size(arrivals)),'ro');

%%
data = readmatrix("data/waveform_early.csv");
d = (data(:,1)+j*data(:,2));
A = readmatrix("data/interarrival_times_early.csv");
A = unique(A,'stable');

arrivals = zeros(size(A)-1);
arrivals(1) = A(1);

for i = 2:length(A)-1
    arrivals(i) = arrivals(i - 1) + A(i);
end

%arrivals_raw = zeros(size(A)-1);
%arrivals_raw(1) = A(1);

%for i = 2:length(A)-1
%    arrivals_raw(i) = arrivals_raw(i - 1) + A(i);
%end


figure(); grid on;
plot(real(d));
hold on;
title("Interference Packets")
plot(arrivals,zeros(size(arrivals)),'ro');

%figure(); grid on;
%plot(real(d));
%hold on;
%plot(arrivals_raw,zeros(size(arrivals_raw)),'ro');
%title("arrival rates");
%%
% Define the rate parameter (lambda)
lambda = 1 / 200;

% Generate a vector of 1,000,000 Exponentially distributed random numbers
n_points = 1000000;
exponential_data = exprnd(lambda, n_points, 1);  % The mean is 1/lambda

% Display the first few values (optional)
disp(min(exponential_data));  % Display first 10 values
figure();
histogram(exponential_data)