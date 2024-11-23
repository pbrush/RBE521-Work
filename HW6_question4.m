clear, clc, close all;
%% Vars
l1 = .020;
l2 = .050;
l3 = .100;
b = 0.75;
v = 0.04; % desired velocity of COG (m/sec)
u = v / (1 - b); % average swing velocity
L = .16; % Stride Length
T = L / v; % Cycle Period
Tt = (1 - b) * T;
Ts = b * T;
timesteps = 20;
A = 1;
omega = 2*pi/L;
horizontal_intervals = linspace(0, 0.16, timesteps);
t0 = 0;
t = linspace(0, Tt, timesteps);

% Init problem relevant variables
xdotmaxf_g = .16;
zdotmaxf_g = xdotmaxf_g;
xf_g = linspace(-L, L, timesteps);
zf_g = A * sin(omega * xf_g - pi / 2);
offset = min(zf_g);
zf_g = zf_g - offset;
h=.1; 
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view
alphaH=[-30*pi/180, 30*pi/180, -90*pi/180, 90*pi/180, -150*pi/180, 150*pi/180];
xb_g(1,1)=-((1-b)/2)*L-D*cos(alphaH(1)); % frame b is a frame attached to the hip

% x pose w.r.t. body
xb_g(2,1)=-((1-b)/2)*L-D*cos(alphaH(2));
xb_g(3,1)=-((1-b)/2)*L;
xb_g(4,1)=-((1-b)/2)*L;
xb_g(5,1)=-((1-b)/2)*L-D*cos(alphaH(5));
xb_g(6,1)=-((1-b)/2)*L-D*cos(alphaH(6));

% z pose w.r.t. body
zb_g(1,1)=h;
zb_g(2,1)=h;
zb_g(3,1)=h;
zb_g(4,1)=h;
zb_g(5,1)=h;
zb_g(6,1)=h;
dt=Tt/5;
xdotb_g=v;
zdotb_g=0;
xdotmaxf_g=.16;
zdotmaxf_g=xdotmaxf_g;

% Calc x and z w.r.t. body frame
for i=1:6
    for t=1:(timesteps-1)
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:timesteps
        xf_b(i,t)=xf_g(t)-xb_g(t);
        zf_b(i,t)=zf_g(t)-zb_g(t);
    end
end

% Calc xdot and zdot
xdotf_g = zeros(1,20);
zdotf_g = zeros(1,20);
for i = 2:20
    xdotf_g(i) = xf_g(i)-xf_g(i-1);
    zdotf_g(i) = zf_g(i)-zf_g(i-1);
end
ave_xdot = mean(xdotf_g);

% Reset t
t = linspace(0, Tt, timesteps);

%% Plotting
subplot(3,2,1)
plot(t,xdotf_g)
title("Horizonal velocity through time")
subplot(3,2,2)
plot(t,zdotf_g)
title("Vertical velocity through time")
subplot(3,2,3)
plot(t,xf_g)
title("Horizonal Movement through time")
subplot(3,2,4)
plot(t,zf_g)
title("Vertical Movement through time")
subplot(3,2,5)
plot(xf_g, zf_g)
title("Horizontal Movement vs Vertical Movement w.r.t ground")
subplot(3,2,6)
plot(xf_b(4,:),zf_b(4,:))
title("Horizontal Movement vs Vertical Movement w.r.t body")

%% Real outputs for each leg
n = 6; % number of legs

for i = 1:n
    xf_g(i,:) = xf_b(1,:);
    zf_b(i,:) = zf_b(1,:);
end

