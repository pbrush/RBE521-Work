function twist_inB = adjoint(twist_inA,T_AB)
    p = T_AB(1:3, 4);
    R = T_AB(1:3, 1:3);
    skp = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0;];
    zero = zeros(3);
    Ad = [R, zero; skp*R R];
    twist_inB = Ad*twist_inA;
end