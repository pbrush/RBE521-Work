function AdT = Adjoint(T)
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    skp = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0;];
    zero = zeros(3);
    AdT = [R, zero; skp*R R];
end