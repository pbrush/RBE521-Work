function R = axisangle2rot(omega,theta)
    c = cos(theta);
    s = sin(theta);
    I = eye(3);
    skomega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0;];
    R = I + s*skomega + (1-c)*skomega^2;
end