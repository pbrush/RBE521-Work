clc, clear, close all

%% Script

% Init variables
hw5_P = [0 10 100 5 5 0]';
hw5_l = [20 70 100];
convention = "XYZ";

% Call the function
hw5_angles = IK(hw5_P, hw5_l, convention);
disp(hw5_angles)

%% Function definition
function angles = IK(P, l, conv)
    % P = 6x1;
    % l = 1x3;
    % conv = string;

    % Physical Parameters
    Rm = 300/2; % Radius of Top, b/2
    Rf = 480/2; % Radius of Base, d/2
    ang1 = 60 * pi / 180; % old alpha
    ang2 = 60 * pi / 180; % old beta
    
    % Extracting limb lengths
    coxa = l(1);
    femur = l(2);
    tibia = l(3);

    % Extracting x,y,z and angles a,b,c
    o = P(1:3,1);
    a = P(4) * pi / 180;
    b = P(5) * pi / 180;
    c = P(6) * pi / 180;

    % Handling XYZ vs ZYZ
    if strcmp(conv, 'ZYZ')
        r1 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; % R1, a about Z
    elseif strcmp(conv, 'XYZ')
        r1 = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];  % R1, a about X
    else
        return
    end
    r2 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)]; % R2, b about Y
    r3 = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1]; % R3, c about Z
    
    % Combining rotation matrices
    R = r1 * r2 * r3;
    
    % Calculating s1
    s11 = [Rm * cos(ang2 / 2), Rm * sin(ang2 / 2), 0]';
    s12 = [-Rm * sin(pi / 6 - ang2 / 2), Rm * cos(pi / 6 - ang2 / 2), 0]';
    s13 = [-Rm * sin(pi / 6 + ang2 / 2), Rm * cos(pi / 6 + ang2 / 2), 0]';
    s14 = [-Rm * cos(pi / 3 - ang2 / 2), -Rm * sin(pi / 3 - ang2 / 2), 0]';
    s15 = [-Rm * cos(pi / 3 + ang2 / 2), -Rm * sin(pi / 3 + ang2 / 2), 0]';
    s16 = [Rm * cos(ang2 / 2), -Rm * sin(ang2 / 2), 0]';
    
    % Concatenate
    s1 = [s11 s12 s13 s14 s15 s16];
    
    % Calculate u
    u1 = [Rf * cos(ang1 / 2), Rf * sin(ang1 / 2), 0]';
    u2 = [-Rf * sin(pi/6 - ang1 / 2), Rf * cos(pi / 6 - ang1 / 2), 0]';
    u3 = [-Rf * sin(pi/6 + ang1 / 2), Rf * cos(pi / 6 + ang1 / 2), 0]';
    u4 = [-Rf * cos(pi / 3 - ang1 / 2), -Rf * sin(pi / 3 - ang1 / 2), 0]';
    u5 = [-Rf * cos(pi / 3 + ang1 / 2), -Rf * sin(pi / 3 + ang1 / 2), 0]';
    u6 = [Rf * cos(ang1 / 2), -Rf * sin(ang1 / 2), 0]';
    
    % Concatenate    
    u = [u1 u2 u3 u4 u5 u6];
    
    % Initialize variables
    Legs = zeros(3,6);      % li in the notes
    length = zeros(6,1);    % _ in the notes
    n = zeros(3,6);

    % Calculate leg vector,s lengths, and unit vectors
    for i = 1:6
        Legs(:,i) = o + R * s1(:, i) - u(:, i);
        length(i) = norm(Legs(:,i), 2);
        n(:, i) = Legs(:,i)/length(i);
    end

    % Init variables for calculating angles
    s2 = zeros(size(s1));
    alpha = zeros(1,6);
    beta = zeros(1,6);
    gamma = zeros(1,6);

    % Loop to calculate intermediary angles and lengths, as well as final angles
    for i = 1:6
        % Calc alpha
        alpha(i) = atan2( Legs(2,i), Legs(1,i) ); % maybe just atan?

        % Calc s2
        s2(1,i) = s1(1,i) + (-1)^i * coxa * cos(alpha(i));
        s2(2,i) = s1(2,i) + (-1)^i * coxa * sin(alpha(i));
        s2(3,i) = s1(3,i);

        % Calc leg length from end of coxa
        l_i = o + R * s2(:, i) - u(:, i);   % li prime

        % Calc magnitude
        length_i = norm(l_i, 2);    % li prime magnitude

        % Init intermediary hegiht variables
        h = Legs(3,i);
        h_prime = l_i(3);

        % Calc intermediary angles
        phi = asin((h_prime - h) / coxa);
        rho = atan2(h_prime, (sqrt( l_i(1)^2 + l_i(2)^2 )));

        % Calc final angles
        beta(i) = acos( (femur^2 + length_i^2 - tibia^2) / (2 * femur * length_i) ) - (rho + phi);
        gamma(i) = pi - cos( (femur^2 + tibia^2 - length_i^2 ) / (2 * femur * tibia));
    end

    % Concatenate
    angles = [alpha; beta; gamma];
end