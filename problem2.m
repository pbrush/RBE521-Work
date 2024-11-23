P = [10 0 100 5 5 0]'; % 6x1 position vector

Rm = 300/2; % Radius of Top, b/2
Rf = 500/2; % Radius of Base, d/2
alpha = 60 * pi / 180;
beta = 60 * pi / 180;

o = P(1:3,1);
a = P(4) * pi / 180;
b = P(5) * pi / 180;
c = P(6) * pi / 180;
% r1 = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];  % R1, a about X
r1 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; % R1, a about Z
r2 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)]; % R2, a about Y
r3 = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1]; % R3, a about Z

R = r1 * r2 * r3;
% R = r1 * r2 * r3;

s1 = [Rm * cos(beta / 2), Rm * sin(beta / 2), 0]';
s2 = [-Rm * sin(pi / 6 - beta / 2), Rm * cos(pi / 6 - beta / 2), 0]';
s3 = [-Rm * sin(pi / 6 + beta / 2), Rm * cos(pi / 6 + beta / 2), 0]';
s4 = [-Rm * cos(pi / 3 - beta / 2), -Rm * sin(pi / 3 - beta / 2), 0]';
s5 = [-Rm * cos(pi / 3 + beta / 2), -Rm * sin(pi / 3 + beta / 2), 0]';
s6 = [Rm * cos(beta / 2), -Rm * sin(beta / 2), 0]';

s = [s1 s2 s3 s4 s5 s6];
% s = [s1 s6];

u1 = [Rf * cos(alpha / 2), Rf * sin(alpha / 2), 0]';
u2 = [-Rf * sin(pi/6 - alpha / 2), Rf * cos(pi / 6 - alpha / 2), 0]';
u3 = [-Rf * sin(pi/6 + alpha / 2), Rf * cos(pi / 6 + alpha / 2), 0]';
u4 = [-Rf * cos(pi / 3 - alpha / 2), -Rf * sin(pi / 3 - alpha / 2), 0]';
u5 = [-Rf * cos(pi / 3 + alpha / 2), -Rf * sin(pi / 3 + alpha / 2), 0]';
u6 = [Rf * cos(alpha / 2), -Rf * sin(alpha / 2), 0]';

u = [u1 u2 u3 u4 u5 u6];
% u = [u1 u6];

for i = 1:6
    Legs(:,i) = o + R * s(:, i) - u(:, i);
    length(i) = norm(Legs(:,i), 2);
end
disp(length)

%% Output
% XYZ: 132.0922  151.6577  161.0500  151.0929  133.3069  123.8025
% ZYZ: 128.3063  141.8588  156.4394  157.3413  143.6902  129.2379