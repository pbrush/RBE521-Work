function [length, n, R] = IK2(P, s, u, conv)
    
    o = P(1:3,1);
    a = P(4) * pi / 180;
    b = P(5) * pi / 180;
    c = P(6) * pi / 180;
    if strcmp(conv, 'ZYZ')
        r1 = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; % R1, a about Z
    elseif strcmp(conv, 'XYZ')
        r1 = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];  % R1, a about X
    else
        return
    end
    r2 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)]; % R2, b about Y
    r3 = [cos(c) -sin(c) 0; sin(c) cos(c) 0; 0 0 1]; % R3, c about Z
    
    R = r1 * r2 * r3;
    
    for i = 1:6
        Legs(:,i) = o + R * s(:, i) - u(:, i);
        length(i) = norm(Legs(:,i), 2);
        n(:, i) = Legs(:,i)/length(i);
    end
end