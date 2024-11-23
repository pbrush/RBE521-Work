close all

rng default % for reproducibility
d = linspace(0,3);

% y = exp(-1.3*d) + 0.05*randn(size(d));
% fun = @(r)exp(-d*r)-y;
% x0 = 4;

y = randn(size(d)) + exp(-1.3*d) + 0.05*randn(size(d));
fun = @(r)r(1) + r(2)*exp(-d*r(2))-y;
x0 = [4,1];

x = lsqnonlin(fun,x0);

disp(x)

% plot(d,y,'ko',d,exp(-x*d),'b-')
% legend('Data','Best fit')
% xlabel('t')
% ylabel('exp(-tx)')