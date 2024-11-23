clc, clear, close all

%% Variables representing physical dimensions of the robot

% Initialize Relevant Variables
Rm = 250/2; % Radius of Top, b/2
Rf = 650/2; % Radius of Base, d/2
alpha = 40 * pi / 180;
beta = 85 * pi / 180;
lmin =  604.8652;
lmax = 1100;
height = 800;


%% Calculations for Part A: Show intersection of COW spheres and plane at Z= 800mm

% Calculate S vectors
s1 = [Rm * cos(beta / 2), Rm * sin(beta / 2), 0]';
s2 = [-Rm * sin(pi / 6 - beta / 2), Rm * cos(pi / 6 - beta / 2), 0]';
s3 = [-Rm * sin(pi / 6 + beta / 2), Rm * cos(pi / 6 + beta / 2), 0]';
s4 = [-Rm * cos(pi / 3 - beta / 2), -Rm * sin(pi / 3 - beta / 2), 0]';
s5 = [-Rm * cos(pi / 3 + beta / 2), -Rm * sin(pi / 3 + beta / 2), 0]';
s6 = [Rm * cos(beta / 2), -Rm * sin(beta / 2), 0]';

% Concatenate
s = [s1 s2 s3 s4 s5 s6];
s = s(1:2, :); % Truncate Z bc it is 0

% Calculate U vectors
u1 = [Rf * cos(alpha / 2), Rf * sin(alpha / 2), 0]';
u2 = [-Rf * sin(pi/6 - alpha / 2), Rf * cos(pi / 6 - alpha / 2), 0]';
u3 = [-Rf * sin(pi/6 + alpha / 2), Rf * cos(pi / 6 + alpha / 2), 0]';
u4 = [-Rf * cos(pi / 3 - alpha / 2), -Rf * sin(pi / 3 - alpha / 2), 0]';
u5 = [-Rf * cos(pi / 3 + alpha / 2), -Rf * sin(pi / 3 + alpha / 2), 0]';
u6 = [Rf * cos(alpha / 2), -Rf * sin(alpha / 2), 0]';

% Concatenate
u = [u1 u2 u3 u4 u5 u6];
u = u(1:2, :); % Truncate Z bc it is 0

% Calculate centers of COW spheres
c = u-s;
cx = c(1,:);
cy = c(2,:);

% Helper variables for graphing circles
x = 0; y = 0;
origin = zeros(1,size(s,2));

% Calculate circle representing top
r1 = Rm;
th = 0:pi/50:2*pi;
xunit1 = r1 * cos(th) + x;
yunit1 = r1 * sin(th) + y;

% Calculate circle representing base
r2 = Rf;
xunit2 = r2 * cos(th) + x;
yunit2 = r2 * sin(th) + y;

% Calcualte Circle Representing COWs at Z=800mm
rl = sqrt(lmax^2-height^2);
cows = zeros(2,6*101);
for i = 1:6
    cows(1,((i-1)*101+1):(i*101)) = rl * cos(th) + cx(i);
    cows(2,((i-1)*101+1):(i*101)) = rl * sin(th) + cy(i);
end

% Find Intersection Points Between COWs
xout = zeros(6,2);
yout = zeros(6,2);
for i = 1:6
    if i + 1 <= 6
        [xout(i, :),yout(i, :)] = circcirc(cx(i),cy(i),rl,cx(i+1),cy(i+1),rl);
    else
        [xout(i, :),yout(i, :)] = circcirc(cx(i),cy(i),rl,cx(1),cy(1),rl);
    end
end

% Determine Which Point of Intersection is Closest by Minimizing Euclidean
% Distance
dist = zeros(6,2);
intersections = zeros(6,2);
for i = 1:6
    for j = 1:2
        dist(i, j) = sqrt(xout(i,j)^2 + yout(i,j)^2);
    end
    if dist(i,1) <= dist(i,2)
        intersections(i, :) = [xout(i, 1), yout(i,1)];
    else
        intersections(i, :) = [xout(i, 2), yout(i,2)];
    end
end

% Manually Truncating the COWs between intersections, and angles between
% intersections
% This is unreasonably long %
i1 = intersections(1,:);
i2 = intersections(2,:);
i3 = intersections(3,:);
i4 = intersections(4,:);
i5 = intersections(5,:);
i6 = intersections(6,:);

% Angles of intersections
i1_ang = atan2(i1(2),i1(1)) + 2*pi;
i2_ang = atan2(i2(2),i2(1)) + 2*pi;
i3_ang = atan2(i3(2),i3(1)) + 2*pi;
i4_ang = atan2(i4(2),i4(1));
i5_ang = atan2(i5(2),i5(1));
i6_ang = atan2(i6(2),i6(1));

% Difference of angles
i1_i6 = i1_ang - i6_ang;
i1_i2 = i1_ang - i2_ang;
i2_i3 = i2_ang - i3_ang;
i3_i4 = i3_ang - i4_ang;
i4_i5 = i4_ang - i5_ang;
i5_i6 = i5_ang - i6_ang;

% Constrain COW segments based on intersection points
cow1 = cows(:,1:101);
cow1_seg_ind = find(cows(2,1:101) >= i1(2) & cows(2,1:101) < i6(2) & cows(1,1:101) >= i6(1) & cows(1,1:101) < i1(1));
seg1_sz = size(cow1_seg_ind);
seg1 = zeros(2, seg1_sz(2));
for i = 1:seg1_sz(2)
    seg1(:, i) = cow1(:, cow1_seg_ind(i));
end

cow2 = cows(:,102:202);
cow2_seg_ind = find(cows(2,102:202) <= 0 & cows(1,102:202) >= i1(1) & cows(1,102:202) <= i2(1));
seg2_sz = size(cow2_seg_ind);
seg2 = zeros(2, seg2_sz(2));
for i = 1:seg2_sz(2)
    seg2(:, i) = cow2(:, cow2_seg_ind(i));
end

cow3 = cows(:,203:303);
cow3_seg_ind = find(cows(2,203:303) <= 0 & cows(1,203:303) >= i2(1) & cows(1,203:303) <= i3(1));
seg3_sz = size(cow3_seg_ind);
seg3 = zeros(2, seg3_sz(2));
for i = 1:seg3_sz(2)
    seg3(:, i) = cow3(:, cow3_seg_ind(i));
end

cow4 = cows(:,304:404);
cow4_seg_ind = find(cows(2,304:404) >= 0 & cows(1,304:404) >= i4(1) & cows(1, 304:404) <= i3(1));
seg4_sz = size(cow4_seg_ind);
seg4 = zeros(2, seg4_sz(2));
for i = 1:seg4_sz(2)
    seg4(:, i) = cow4(:, cow4_seg_ind(i));
end

cow5 = cows(:,405:505);
cow5_seg_ind = find(cows(2,405:505) >= 0 & cows(1,405:505) >= i5(1) & cows(1, 405:505) <= i4(1));
seg5_sz = size(cow5_seg_ind);
seg5 = zeros(2, seg5_sz(2));
for i = 1:seg5_sz(2)
    seg5(:, i) = cow5(:, cow5_seg_ind(i));
end

cow6 = cows(:,506:606);
cow6_seg_ind = find(cows(2,506:606) >= 0 & cows(1,506:606) >= i6(1) & cows(1, 506:606) <= i5(1));
seg6_sz = size(cow6_seg_ind);
seg6 = zeros(2, seg6_sz(2));
for i = 1:seg6_sz(2)
    seg6(:, i) = cow6(:, cow6_seg_ind(i));
end

% This is unreasonably long but creating the loop for it hurt more%

% Concatenate
segments = [i6' seg1 i1' seg2 i2' seg3 i3' seg4 i4' seg5 i5' seg6 i6'];
pad = zeros(1, 83); % Size(segments) = 2, 83), Creating one more row of zeros for 3D plotting later
segments_3D = [segments; pad];

%% Graphing A: The s and u vectors, the base and ee circles, and centers of the relevant COW spheres, and the truncated COW segments bounded by their intersection points

figure(1)
title("Part A")
hold on
% S and U vectors
plot([origin; s(1,:)],[origin; s(2,:)]);
plot([origin; u(1,:)],[origin; u(2,:)]);

% Centers of COW spheres
plot(cx, cy, "o");

% EE & Base, respectively
plot(xunit1, yunit1);
plot(xunit2, yunit2);

% Intersection of Top 6 COWs and plane at 800mm, and Intersections of Those
% Circles
for i = 1:6
    % plot(cows(1, ((101*(i-1))+1):(101*i)), cows(2, ((101*(i-1))+1):(101*i)));
    plot(intersections(i, 1), intersections(i, 2))
end

intersections = intersections';

plot(segments(1,:), segments(2,:), "-")
hold off

%% Calculations for Part B: Creating the mesh of points 5mm apart in X and Y

% Mesh Min and Max values
xsegmax = max(segments(1,:));
xsegmin = min(segments(1,:));
ysegmax = max(segments(2,:));
ysegmin = min(segments(2,:));

% Interpolate between Mins and Max's
mesh_unbound_x = xsegmin:5:xsegmax;
mesh_unbound_y = ysegmin:5:ysegmax;

% Get number of rows and columns of mesh
mesh_rows = size(mesh_unbound_x);
mesh_cols = size(mesh_unbound_y);

% Create grid from interpolated sets
[X, Y] = meshgrid(mesh_unbound_x, mesh_unbound_y);

% Use Circles to create Masks
X_row = X(1,:);
Y_col = Y(:,1);

% Create the masks
mask1 = (X_row-cx(1)).^2 + (Y_col-cy(1)).^2 <= rl^2;
mask2 = (X_row-cx(2)).^2 + (Y_col-cy(2)).^2 <= rl^2;
mask3 = (X_row-cx(3)).^2 + (Y_col-cy(3)).^2 <= rl^2;
mask4 = (X_row-cx(4)).^2 + (Y_col-cy(4)).^2 <= rl^2;
mask5 = (X_row-cx(5)).^2 + (Y_col-cy(5)).^2 <= rl^2;
mask6 = (X_row-cx(6)).^2 + (Y_col-cy(6)).^2 <= rl^2;

% Concatenate
mask = mask1 & mask2 & mask3 & mask4 &mask5 &mask6;

% Bound mesh to intersections
mesh_x = X(mask);
mesh_y = Y(mask);

%% Graphing Part B: The COW segments and the grid mesh
figure(2)
title("Part B")
hold on

% Edges
plot(segments(1,:), segments(2,:), "-")

% Bounded Grid Mesh
scatter(mesh_x, mesh_y)
hold off

%% Part C: Create Forward Kinematic Error Model and calculate error at each point on the mesh

%                    six        siy siz   uix       uiy   uix   lio
nominal_params = [ 92.1597   84.4488 0  305.4001  111.1565 0 604.8652 27.055   122.037 0  -56.4357  320.0625 0 604.8652 -119.2146   37.5882 0 -248.9644  208.9060 0 604.8652 -119.2146  -37.5882 0 -248.9644 -208.9060 0 604.8652 27.055  -122.037 0  -56.4357 -320.0625 0 604.8652 92.1597  -84.4488 0  305.4001 -111.1565 0 604.8652]';

simulated_params =  [ 96.6610   81.7602  1.0684   305.2599  115.0695  2.6219 604.4299 22.2476  125.2511 -0.05530  -55.2814  322.9819  4.2181 607.2473 -122.4519   36.6453  4.3547  -244.7954  208.0087  3.9365 600.4441 -120.6859  -34.4565 -4.9014  -252.5755 -211.8783 -3.0128 605.9031 24.7769 -125.0489 -4.8473   -53.9678 -320.6115  4.3181 604.5251 91.3462  -80.9866  0.2515   302.4266 -109.4351  3.3812 600.0616]';

% Forward Kinematic Error Model Constants
drho = simulated_params-nominal_params;
ddl = 0;

% Loop Constants
bound_mesh_size = size(mesh_x);
bound_mesh_size = bound_mesh_size(1);

% FKEM Output init
dP = zeros(6, bound_mesh_size);
errors = zeros(1, bound_mesh_size);

% Loop thorugh each point in the mesh and calculate dP and Error
for j = 1:bound_mesh_size
    P = [mesh_x(j), mesh_y(j), height, 0, 0, 0]';
    [length, n, R, s] = IK(P, "XYZ");
    Jv = parallel_jacob0(P, "XYZ");
    Jrho = zeros(6, 42);
    
    % Create Jrho from Nominal Parameters
    for i = 1:6
        Jrho(i, (((i-1)*7)+1):(7*i)) = [n(:,i)'*R -n(:,i)' -1];
    end
    
    % Calculate dP and errors
    dP(:, j) = pinv(Jv) * (ddl - Jrho*drho);
    errors(1, j) = sqrt(dP(1,j)^2 + dP(2,j)^2 + dP(3,j)^2);
end

%% Graphing Part C: Mesh of the errors
figure(3)
scatter3(mesh_x, mesh_y, errors, 5, errors)
hold on
plot3(segments_3D(1,:), segments_3D(2,:), segments_3D(3, :), "-")
title("Part C")
