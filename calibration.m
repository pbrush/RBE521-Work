clc, clear, close all

%% Variables representing physical dimensions of the robot

% Initialize Relevant Variables
Rm = 250.9339/2; % Radius of Top, b/2
Rf = 634.2376/2; % Radius of Base, d/2
alpha = 40 * pi / 180;
beta = 85 * pi / 180;
lmin =  604.8652;
lmax = 1100;
height = 800;
conv = "XYZ";

% Nominal Values
global nominal_params
nominal_params = [ 92.1597   84.4488 0  305.4001  111.1565 0 604.8652 27.055   122.037 0  -56.4357  320.0625 0 604.8652 -119.2146   37.5882 0 -248.9644  208.9060 0 604.8652 -119.2146  -37.5882 0 -248.9644 -208.9060 0 604.8652 27.055  -122.037 0  -56.4357 -320.0625 0 604.8652 92.1597  -84.4488 0  305.4001 -111.1565 0 604.8652]';

% "Real" values for FK
real_params =  [ 96.6610   81.7602  1.0684   305.2599  115.0695  2.6219 604.4299 22.2476  125.2511 -0.05530  -55.2814  322.9819  4.2181 607.2473 -122.4519   36.6453  4.3547  -244.7954  208.0087  3.9365 600.4441 -120.6859  -34.4565 -4.9014  -252.5755 -211.8783 -3.0128 605.9031 24.7769 -125.0489 -4.8473   -53.9678 -320.6115  4.3181 604.5251 91.3462  -80.9866  0.2515   302.4266 -109.4351  3.3812 600.0616]';

% Create s, u, & l real
s_real = zeros(3,6);
u_real = zeros(3,6);
l_real = zeros(1,6);

% Populate s, u & l real
for k = 1:6
    s_real(1, k) = real_params(1+(7*(k-1)));
    s_real(2, k) = real_params(2+(7*(k-1)));
    s_real(3, k) = real_params(3+(7*(k-1)));
    u_real(1, k) = real_params(4+(7*(k-1)));
    u_real(2, k) = real_params(5+(7*(k-1)));
    u_real(3, k) = real_params(6+(7*(k-1))); 
    l_real(1, k) = real_params(7*k);
end

% Choose "m" configurations
% Points of intersections of COW's with plane at 800 mm, and a point in the center that is high in the workspace 
hw3_configurations =  [-302.7697  -16.067   270.6352  438.0873  605.5394  438.0873  270.6352 -16.067 -302.7697 -422.0201 -541.2705;
                       -524.4125 -496.5833 -468.7540 -234.3770  0.0000    234.3770  496.5833 468.7540  524.4125 262.2063  0.0000  ;
                        800.0     800.0     800.0     800.0     800.0     800.0     800.0    800.0     800.0    800.0     800.0    ];

% For the sake of simplicity, I chose the 6 intersection points of the
% COW spheres that intersect with the plane at 800mm, and linearly
% interpolated a point between the adjacent points. While it is probably
% best practice to use points at varying Z values, I ran out of time to
% experiment with different points.


% Extract size for incrementing
hw3_c_size = size(hw3_configurations);
m = hw3_c_size(2);

% Initialize vars for later
global O_real
O_real = zeros(3, m);
global R_real
R_real = zeros(3, 3*m);
global dl
dl = zeros(6, m);
global cost_fun
cost_fun = zeros(6, m);

% Set which configurations I use for better readability
c = hw3_configurations;

disp("Ignore the display output here, I have literally no idea what they are resulting from.")

%% Main Loop
for j = 1:m

    % Extract position from configuration & create P vector
    position = c(:, j)';
    P = [position 0 0 0]';

    % Get Leg Lengths from IK
    [l, n, R] = IK2(P, s_real, u_real, conv);
    lg = l';
    % Inject known noise into the FK
    P_real = fk2(P, lg, s_real, u_real, conv);
    O_real(1:3,j) = P_real(1:3,1);
    R_lb = 1 + (3*(j-1));
    R_ub = 3 + (3*(j-1));
    R_real(1:3,R_lb:R_ub) = R;

    % Cost function is created in helper function
    % and fed into lsqnonlin
end


%% Use least squares to find parameters

% For displaying iteration information
disp_opts = optimoptions('lsqnonlin', 'Display', 'iter');

% Set initial guess for better readability
initial_guess = nominal_params;

% Minimize cost function using lsqnonlin
[sim_params, resnorm, residual, exitflag, output] = lsqnonlin(@calib_cost_function, initial_guess, [], [], disp_opts);

%% Graphing

% Get pre and post calibration errors
pre_calib_errors = abs(nominal_params - real_params);
post_calib_errors = abs(nominal_params - sim_params);

% Concatenate
z = [post_calib_errors pre_calib_errors];

% Graph
bar3(z)
ylabel("Error (mm)")
xlabel("Kinematics Parameters")
title("Identification Results")
legend("Post Calibration Results", "Pre Calibration Results")


%% Helper function for determining cost
function res = calib_cost_function(params)
    global m
    global R_real
    global cost_fun

    % Local var inits
    cost_fun = zeros(6,m);
    s_real = zeros(3,6);
    u_real = zeros(3,6);
    l_real = zeros(1,6);

    % Create vars from params to avoid rewriting code and making it
    % readable
    for k = 1:6
        conf_cost = 0;
        s_real(1, k) = params(1+(7*(k-1)));
        s_real(2, k) = params(2+(7*(k-1)));
        s_real(3, k) = params(3+(7*(k-1)));
        u_real(1, k) = params(4+(7*(k-1)));
        u_real(2, k) = params(5+(7*(k-1)));
        u_real(3, k) = params(6+(7*(k-1))); 
        l_real(1, k) = params(7*k);
    end

    % Create cost function
    for j = 1:m
        for i = 1:6
            R_lb = 1 + (3*(j-1));
            R_ub = 3 + (3*(j-1));
            O = O_real(1:3,j);
            s = s_real(:,i);
            u = u_real(:, i);
            R = R_real(1:3,R_lb:R_ub);
            cost_fun(:, j) = ((O+R*s-u)'*(O+R*s-u) - (l_real-l).^2).^2;
        end
    end
    res = cost_fun;
end