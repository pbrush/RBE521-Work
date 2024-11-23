function T = twist2ht(S,theta)
    omega = S(1:3,:);
    v = S(4:6,:);
    I = eye(3);
    skomega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0;];
    p = (I*theta + (1-cos(theta))*skomega + (theta-sin(theta))*skomega^2)*v;
    R = axisangle2rot(omega,theta);
    
    T = [ R p; 0 0 0 1;];
end