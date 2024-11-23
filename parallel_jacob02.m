function J = parallel_jacob02(P, s, u, conv)
    [l, n, R] = IK2(P, s, u, conv);
    J = [n', cross(R*s, n)'];
end