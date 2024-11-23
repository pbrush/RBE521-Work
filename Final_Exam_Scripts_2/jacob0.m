function J = jacob0(S,q)
    [m, n] = size(S);
    J = S(:,1);
    for i = 2:n
        S_i = S(:, i);
        T = eye(4);
        for j = 1:(i-1)
            S_j = S(:, j);
            T = T * twist2ht(S_j, q(j));
        end
        J = [J, adjoint(S_i,T)];
    end
end