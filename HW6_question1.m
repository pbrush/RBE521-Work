clc, clear, close all

%% Script
beta = 0.5:0.01:1;
v = 0:0.05:1;
beta_size = size(beta);
loop_range = beta_size(2);

% hold on
for i = 1:loop_range
    u = beta(i)/(1-beta(i))*v;
    plot(v, u)
end
title("v(t) with beta values ranging from 0.5 to 1 in 0.05 increments")
xlabel("u(t)")
ylabel("v(t)")
% legend("0.50", "0.55", "0.60", "0.65", "0.70", "0.75", "0.80", "0.85", "0.90", "0.95", "1.00")
% As beta increases, the slope of v(t) increases.

