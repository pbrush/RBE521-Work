function p = fk2(P0, lg, s, u, conv)
    eps = 0.0000001;
    dl = 1;
    P = zeros(6,1);
    P(:,1) = P0;
    i = 2;
    while dl > eps
        disp(dl)
        J = parallel_jacob02(P(:,i-1), s, u, conv); % Step 2
        a = P(4, i-1) * pi /180; % Step 3
        b = P(5, i-1) * pi /180;
        c = P(6, i-1) * pi /180;
        B = [0 -sin(a) sin(b)*cos(a);
             0  cos(a) sin(b)*sin(a);
             1    0       cos(b)    ];
        if strcmp(conv, 'ZYZ')
            B = [0 -sin(a) sin(b)*cos(a);
                 0  cos(a) sin(b)*sin(a);
                 1    0       cos(b)    ];
        elseif strcmp(conv, 'XYZ')
            B = [1    0         sin(b);
                 0  cos(a)  -sin(a)*cos(b);
                 0  sin(a)  cos(a)*cos(b)];
        else
            return
        end

        T = [eye(3)       zeros(3,3);
             zeros(3,3)       B     ];
        
        % Step 4
        [l, n, R] = IK2(P(:,i-1), s, u, conv);
        Dl = lg - l'; % lg should be column
        P(:,i) = P(:, i-1) + pinv(J*T) * Dl; % Step 5
        dl = norm(Dl, 2);
        i = i + 1;
    end
    p = P(:, end);
end