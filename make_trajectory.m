function traj = make_trajectory(type, params)
    qi = params.q(1);
    qf = params.q(2);
    vi = params.v(1);
    vf = params.v(2);
    ti = params.t(1);
    tf = params.t(2);
    dt = params.dt;
    t = [ti:dt:tf];
    
    % Determine cubic vs quintic
    if strcmp(type, 'cubic')
        coeff_mtx = [ 1 ti   ti^2   ti^3;
                    0 1    2*ti    3*ti^2; 
                    1 tf   tf^2   tf^3;
                    0 1    2*tf   3*tf^2];
        bounds = [qi; vi; qf; vf];
        A = inv(coeff_mtx) * bounds;
        q = A(1) + A(2)*t + A(3)*t.^2 + A(4)*t.^3;
        v = A(2) + 2*A(3)*t + 3*A(4)*t.^2;
        a = 2*A(3) + 6*A(4)*t;
    elseif strcmp(type, 'quintic')
        ai = params.a(1);
        af = params.a(2);
        coeff_mtx = [1 ti   ti^2    ti^3    ti^4    ti^5;
                         0 1    2*ti    3*ti^2  4*ti^3  5*ti^4;
                         0 0    2       6*ti    12*ti^2 20*ti^3;
                         1 tf   tf^2    tf^3    tf^4    tf^5;
                         0 1    2*tf    3*tf^2  4*tf^3  5*tf^4;
                         0 0    2       6*tf    12*tf^2 20*tf^3];
        bounds = [qi; vi; ai; qf; vf; af];
        A = inv(coeff_mtx) * bounds;
        q = A(1) + A(2)*t + A(3)*t.^2 + A(4)*t.^3 + A(5)*t.^4 + A(6)*t.^5;
        v = A(2) + 2*A(3)*t + 3*A(4)*t.^2 + 4*A(5)*t.^3 + 5*A(6)*t.^4;
        a = 2*A(3) + 6*A(4)*t + 12*A(5)*t.^2 + 20*A(6)*t.^3;
    else
         disp("Invalid type")
    end
    traj.t = t;
    traj.q = q;
    traj.v = v;
    traj.a = a;
end