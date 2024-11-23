clc, clear, close all;
%% INITIAL DATA
% Links
l1=.050;
l2=.050;

b = 0.5;

% Average velocity of foot tip in swing phase (0.1)
u = .1; 

% Desired velocity of COG (m/sec)
v = (1-b)/b*u;

% Angular velocity of COG
w = 0; 

% Distance of foot tip to COG or center of rotation
R = 0.05;

% Average swing velocity of the leg wrt the body
% u = (b/(1-b))*v;

% Average swing velocity of the leg wrt the ground
u_b = v/(1-b);

% Stride Length (COG displacement in one cycle time)
L=.03;

% Cycle Period
T = L/v;
% Transfering time
Tt = (1-b)*T;
% Support time
Ts = b*T;

%% GAIT GENERATION
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+b; % Kinematic phase of leg 3
p(4)=p(2)+b; % Kinematic phase of leg 4

% Get size of p
m = size(p);

% Number of legs
n = m(2);

for j=1:n
    for i=1:n
        if p(i)>=1
            p(i)=p(i)-1;
        end
    end
    j=j+1;
end
p

%% POSITIONAL FOOT TRAJECTORY PLANNING
%TransferRing time is divided to 5 equal extents. Such a selection is completely
%arbitrary.
t0=0;
t1=Tt/5;
t2=2*t1;
t3=Tt-t2;
t4=Tt-t1;
t5=Tt;

Ttt=[t0,t1,t2,t3,t4,t5];

% Everything w.r.t ground frame
% Maximum leg transferring speed.
xdotmaxf_g=2*(t5-t0)*u/((t4-t1)+(t3-t2)); 
% this is the average velocity divided by the surface are under the velocity graph. 

 % Arbitrary.
zdotmaxf_g=xdotmaxf_g;

xdotf_g=[0,0,xdotmaxf_g,xdotmaxf_g,0,0];

for i=1:n
    xf_g(i,1)=-L/2;
    xf_g(i,2)=xf_g(i,1)+0;
    xf_g(i,3)=xf_g(i,2)+(t2-t1)*xdotmaxf_g/2;
    xf_g(i,4)=xf_g(i,3)+(t3-t2)*xdotmaxf_g;
    xf_g(i,5)=xf_g(i,4)+(t4-t3)*xdotmaxf_g/2;
    xf_g(i,6)=xf_g(i,5)+0;
end

zdotf_g=[0,zdotmaxf_g,0,0,-zdotmaxf_g,0];

for i=1:n
    zf_g(i,1)=0;
    zf_g(i,2)=zf_g(i,1)+(t1-t0)*zdotmaxf_g/2;
    zf_g(i,3)=zf_g(i,2)+(t2-t1)*zdotmaxf_g/2;
    zf_g(i,4)=zf_g(i,3)+0;
    zf_g(i,5)=zf_g(i,4)-(t4-t3)*zdotmaxf_g/2;
    zf_g(i,6)=zf_g(i,5)-(t5-t4)*zdotmaxf_g/2;
end

%%%%%%%% Now it's the time to change our cooridinte system properly
%Leg number 4:
% Height of the robot (m). Knees at 45 degrees and l1 = l2, therefore
% 2*length*cos(45)
h=2*l1*cos(pi/2);
% Distance between the foot tip and hip joint from top view, 0 b/c they're
% inline
D=0; 

% Not necessary to change because it D = 0
alphaH = zeros(1, 6);

%% Unsure about these
xb_g(1,1)=-((1-b)/2)*L-D*cos(alphaH(1)); % frame b is a frame attached to the hip joint whose
xb_g(2,1)=-((1-b)/2)*L-D*cos(alphaH(2));
xb_g(3,1)=((1-b)/2)*L; % two in front, two in back
xb_g(4,1)=((1-b)/2)*L;

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

for i=1:n
    for t=1:5 % 5 is because of time dividing to 5 equal extents.
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:6
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end

% ****xf_b(t) and zf_b(t) ARE THE SAME FOR ALL LEGS i=1,2,...,4. But yf_b
% differs as follow. For each leg, yf_b is the same for all t. 
yf_b=zeros(1,4);

for i=1:n % this is becaus of 4 legs
    for j=1:6 % this is because we have divided out T to 6 dt.
        xf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        yf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        zf_H(i,j)=zf_b(i,j);
    end
end
xf_H
yf_H
zf_H
xf_b
yf_b
zf_b
xb_g
%% ----------------------- *** PLOT  DATA *** ---------------------%%
subplot(3,2,1)
plot(Ttt,xdotf_g)
subplot(3,2,2)
plot(Ttt,zdotf_g)

subplot(3,2,3)
plot(Ttt,xf_g(4,:))
subplot(3,2,4)
plot(Ttt,zf_g(4,:))

subplot(3,2,5)
plot(xf_g(4,:),zf_g(4,:))

subplot(3,2,6)
plot(xf_b(4,:),zf_b(4,:))

%% ---------------------------INVERSE KINEMATICS------------------------%%
% Alpha0=0; %hip joint in the home postion
% Beta0=atan(h/l2); %Knee joint in the home postion
for i=1:4 % for all 6 legs
    for j=1:6 % time discrete
        l3(i,j) = sqrt(zf_H(i,j)^2+xf_H(i,j)^2);
        hip(i,j) = acos((l1^2+l2^2-l3(i,j)^2)/(2*l1*l2));
        knee(i, j) = atan2(xf_H(i,j), zf_H(i,j)) - acos((l1^2+l3(i,j)^2-l2^2)/(2*l1*l3(i,j)^2));
    end
end
A=hip*180/pi
B=knee*180/pi
Ttt

%% VELOCITY FOOT TRAJECTORY PLANNING
ydotf_b=0;
for t=1:6
    xdotf_b(t)=xdotf_g(t)-xdotb_g;
    zdotf_b(t)=zdotf_g(t)-zdotb_g;
end
for i=1:n
    for j=1:6
        xdotf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        ydotf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        zdotf_H(i,j)=zdotf_b(j);
    end
end

S = [0 0 0 0 0 1; -l1*cos(pi/4) -l1*sin(pi/4) 0 0 0 1]';

for j = 1:6
    for i = 1:n
        theta1=hip(i,j);
        theta2=knee(i,j);
        q = [theta1, theta2];
        J = jacob0(S,q);
        Thetadot(i,j,:)=pinv(J')*[xdotf_H(i,j); zdotf_H(i,j)]; % [xdotf_H(i,j);ydotf_H(i,j);zdotf_H(i,j)]; removed ydot since it doesn't change
    end
end

Thetadot
xdotb_g
zdotb_g
xdotf_H
ydotf_H
zdotf_H

%% End

t = 1:6;


% Continuous and smooth trajectory between the individual points
for i = 1:4
    for j = 1:5
        knee_params.q = [hip(i,j) hip(i,j+1)]; % how to do knee
        knee_params.v = [Thetadot(i,j,1), Thetadot(i,j+1,1)];
        knee_params.t = [t(j), t(j+1)];
        knee_params.dt = 0.1;
        type = "cubic";
        kneetraj = make_trajectory(type, knee_params);
        kneeq(i, j, :) = kneetraj.q;
        kneev(i, j, :) = kneetraj.v;
        kneetraj_t(i, j, :) = kneetraj.t;

        hip_params.q = [hip(i,j) hip(i,j+1)]; % how to do knee
        hip_params.v = [Thetadot(i,j,2), Thetadot(i,j+1,2)];
        hip_params.t = [t(j), t(j+1)];
        hip_params.dt = 0.1;
        hiptraj = make_trajectory(type, knee_params);
        hipq(i, j, :) = hiptraj.q;
        hipv(i, j,:) = hiptraj.v;
        hiptraj_t(i, j, :) = hiptraj.t;
    end
end
