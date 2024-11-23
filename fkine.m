function T = fkine(S,M,q,frame)
    [m,n] = size(S);
    T = eye(4);
    for i = 1:n
            T = T*twist2ht(S(:, i),q(i));
    end
    if strcmp(frame,'space')
        T = T * M;
    elseif strcmp(frame,'body')
        T = M * T;
    else
        T = T * M;
    end
end