function J = parallel_jacob0(P, conv)
    [l n R s] = IK(P, conv);
    J = [n', cross(R*s, n)'];
end