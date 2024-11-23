clear, clc, close all;
%% ----------------------- *** INITIAL DATA *** ------------------------%%
l1=.020;
l2=.050;
l3=.100;
b=0.75;
v=0.04; % desired velocity of COG (m/sec)
u=v/(1-b); % average swing velocity of the leg wrt the ground
L=.16; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/v; % Cycle Period
Tt=(1-b)*T;
Ts=b*T;

%% ------------- *** POSITIONAL FOOT TRAJECTORY PLANNING *** ------------%%
%Transfering time is divided to 5 equal extents. Such a selection is completely
%arbitrary.
t0=0;
t1=Tt/5;
t2=2*t1;
t3=Tt-t2;
t4=Tt-t1;
t5=Tt;
Ttt=[t0,t1,t2,t3,t4,t5];
%%%%%%%%%%HERE EVERY THING IS IN Gi COORDINATE SYSTEM WHICH IS GROUND
%COORDINATE SYSTEM FOR EACH LEG
% xdotmaxf_g=2*(t5-t0)*u/((t4-t1)+(t3-t2)); %maximum leg transferring speed.
% this is the average velocity divided by the surface are under the velocity graph.
xdotmaxf_g=.16;
zdotmaxf_g=xdotmaxf_g; % it is arbitrary.
xdotf_g=[0,0,xdotmaxf_g,xdotmaxf_g,0,0];
for i=1:6
    xf_g(i,1)=-L/2;
    xf_g(i,2)=xf_g(i,1)+0;
    xf_g(i,3)=xf_g(i,2)+(t2-t1)*xdotmaxf_g/2;
    xf_g(i,4)=xf_g(i,3)+(t3-t2)*xdotmaxf_g;
    xf_g(i,5)=xf_g(i,4)+(t4-t3)*xdotmaxf_g/2;
    xf_g(i,6)=xf_g(i,5)+0;
end
zdotf_g=[0,zdotmaxf_g,0,0,-zdotmaxf_g,0];
for i=1:6
    zf_g(i,1)=0;
    zf_g(i,2)=zf_g(i,1)+(t1-t0)*zdotmaxf_g/2;
    zf_g(i,3)=zf_g(i,2)+(t2-t1)*zdotmaxf_g/2;
    zf_g(i,4)=zf_g(i,3)+0;
    zf_g(i,5)=zf_g(i,4)-(t4-t3)*zdotmaxf_g/2;
    zf_g(i,6)=zf_g(i,5)-(t5-t4)*zdotmaxf_g/2;
end
%%%%%%%% Now it's the time to change our cooridinte system properly
%Leg number 4:
h=.1; %(m) height of the robot. Here h=l3 because it is assumed in the home
% positon.
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view
alphaH=[-30*pi/180, 30*pi/180, -90*pi/180, 90*pi/180, -150*pi/180, 150*pi/180];
xb_g(1,1)=-((1-b)/2)*L-D*cos(alphaH(1)); % frame b is a frame attached to the hip
% joint whose
xb_g(2,1)=-((1-b)/2)*L-D*cos(alphaH(2));
xb_g(3,1)=-((1-b)/2)*L;
xb_g(4,1)=-((1-b)/2)*L;
xb_g(5,1)=-((1-b)/2)*L-D*cos(alphaH(5));
xb_g(6,1)=-((1-b)/2)*L-D*cos(alphaH(6));
% x axis is parallel to the x axis of frame g (Gi).
zb_g(1,1)=h;
zb_g(2,1)=h;
zb_g(3,1)=h;
zb_g(4,1)=h;
zb_g(5,1)=h;
zb_g(6,1)=h;
dt=Tt/5;
xdotb_g=v;
zdotb_g=0;
for i=1:6
    for t=1:5 % 5 is becaus of time dividing to 5 equal extents.
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:6
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end

%% Plot
subplot(3,2,1)
plot(Ttt,xdotf_g)
subplot(3,2,2)
plot(Ttt,zdotf_g)
subplot(3,2,3)
plot(Ttt,xf_g(2,:))
subplot(3,2,4)
plot(Ttt,zf_g(2,:))
subplot(3,2,5)
plot(xf_g(4,:),zf_g(2,:))
title("Horizonal Movement w.r.t ground")
subplot(3,2,6)
plot(xf_b(4,:),zf_b(2,:))
title("Horizonal Movement w.r.t body")

%% Real outputs for each leg
n = 6; % number of legs

for i = 1:n
    xf_g(i,:) = xf_b(1,:);
    zf_b(i,:) = zf_b(1,:);
end