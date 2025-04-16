function [h] = Gaussian_Filter(bt, span, spb)
    filter_len = span * spb + 1;
    t = linspace(-span / 2, span / 2, filter_len);
    alpha = sqrt(log(2) / 2) / bt;
    h = sqrt(pi) / alpha * exp(-(t * pi / alpha).^2);
    h = h / sum(h);
    end